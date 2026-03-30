import datetime
import os

import spidev
import logging
import time
import queue
import threading
from gpiozero import DigitalInputDevice, DigitalOutputDevice

from utility.BGT60TR13C_CONST import *

RET_VAL_OK = 0
RET_VAL_ERR = -1

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class BGT60TR13C:
    def __init__(self, spi_bus=0, spi_dev=0, spi_speed=10_000_000, rst_pin=12, irq_pin=25, version=0, save_to_file=None):
        self.__fifo_num_sampler_per_burst = None
        self.__fifo_num_samples_irq = None
        self.__fifo_num_samples_per_frame = None
        self.__register_config_file_name = None
        self.__spi = spidev.SpiDev()
        self.__spi.open(spi_bus, spi_dev)
        self.__spi.max_speed_hz = spi_speed
        self.__spi.mode = 0
        self.__rst = DigitalOutputDevice(rst_pin)
        self.__irq = DigitalInputDevice(irq_pin, pull_up=False)
        self.hard_reset()
        self.frame_buffer = queue.Queue(maxsize=256) # It is used to hold FMCW frames. Each one in it is a complete frame.
        self.__sub_frame_buffer = [] # It is used to hold incomplete FMCW frames segments, which come from SPI burst read. Once one complete frame is received, it will be cleared. It is in bytes.

        self.__num_samples_per_frame = 0
        self.__num_bytes_per_frame = 0
        self.__num_sampler_per_burst = 0
        self.__last_gsr_reg = 0

        # Data collection thread
        self.__data_collection_thread = None
        self.__data_collection_stop_event = threading.Event()

        self.__version = version
        self.__seq = 0
        self.__save_to_file = save_to_file
        self.__file_fd = None

    def set_fifo_parameters(self, num_samples_per_frame, num_samples_irq, num_sampler_per_burst):
        # Note that regular Raspberry PI spidev has a limit of 4096 bytes per transfer. In addition, the frame size may be larger than buffer size. So 3 parameters are used to control it.
        # num_samples_frame: The number of samples for one FMCW frame.
        # num_samples_irq: The number of samples to trigger the interrupt.
        # num_sampler_burst: The number of samples for each SPI burst read.

        self.__fifo_num_samples_per_frame = num_samples_per_frame
        self.__fifo_num_samples_irq = num_samples_irq
        self.__fifo_num_sampler_per_burst = num_sampler_per_burst


    def __set_fifo_parameters(self):
        logging.info(f"num_samples_per_frame: {self.__fifo_num_samples_per_frame}")
        logging.info(f"num_samples_irq: {self.__fifo_num_samples_irq}")
        logging.info(f"num_sampler_per_burst: {self.__fifo_num_sampler_per_burst}")

        self.__num_samples_per_frame = self.__fifo_num_samples_per_frame
        self.__num_bytes_per_frame = (self.__fifo_num_samples_per_frame>>1)*3
        self.__num_sampler_per_burst = self.__fifo_num_sampler_per_burst
        self.__set_fifo_limit(self.__fifo_num_samples_irq)

    def __set_reg(self, reg_addr, data):
        # Build the SPI register address and data
        temp = (reg_addr << BGT60TRXX_SPI_REGADR_POS) & BGT60TRXX_SPI_REGADR_MSK
        temp |= BGT60TRXX_SPI_WR_OP_MSK  # Set the write operation flag
        temp |= (data << BGT60TRXX_SPI_DATA_POS) & BGT60TRXX_SPI_DATA_MSK

        # Convert data to 4-byte transmission format
        tx_data = [temp >> 24 & 0xFF, temp >> 16 & 0xFF, temp >> 8 & 0xFF, temp & 0xFF]

        # Send data through SPI, and read SPI at the same time
        rx_data = self.__spi.xfer2(tx_data)
        result = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]
        self.__last_gsr_reg = rx_data[0]

        return result

    def __get_reg(self, reg_addr):
        # Build the SPI register address
        temp = (reg_addr << BGT60TRXX_SPI_REGADR_POS) & BGT60TRXX_SPI_REGADR_MSK

        # Convert data to 4-byte transmission format
        tx_data = [temp >> 24 & 0xFF, temp >> 16 & 0xFF, temp >> 8 & 0xFF, temp & 0xFF]

        # Send read command and receive data
        rx_data = self.__spi.xfer2(tx_data)
        result = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]
        self.__last_gsr_reg = rx_data[0]

        return result

    def __get_fifo_data(self, num_samples):
        fifo_data = []
        if 0 < num_samples >> 1 <= BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE and num_samples % 2 == 0:
            tx_data = [BGT60TRXX_SPI_BURST_MODE_CMD >> 24 & 0xFF,
                       BGT60TRXX_REG_FIFO_TR13C << BGT60TRXX_SPI_BURST_MODE_SADR_POS >> 16 & 0xFF,
                       0x00, 0x00] + [0x00] * ((num_samples >> 1) * 3)
            rx_data = self.__spi.xfer2(tx_data)
            self.__last_gsr_reg = rx_data[0]
            if self.check_gsr_reg() != RET_VAL_OK:
                logging.error("GSR Error Detected")
                self.print_gsr_reg()
                self.print_fifo_status()
            else:
                fifo_data = rx_data[4:]
        else:
            logging.error("Invalid num_samples.")
        return fifo_data

    def check_chip_id(self):
        chip_id = self.__get_reg(BGT60TRXX_REG_CHIP_ID)
        logging.info(f"CHIP_ID: 0x{chip_id:08X}")
        chip_id_digital = (chip_id & BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_MSK) >> BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_POS
        chip_id_rf = (chip_id & BGT60TRXX_REG_CHIP_ID_RF_ID_MSK) >> BGT60TRXX_REG_CHIP_ID_RF_ID_POS

        # Determine the device type based on extracted IDs
        if chip_id_digital == 3 and chip_id_rf == 3:
            logging.info("This chip is BGT60TR13C.")
            return RET_VAL_OK
        else:
            logging.info("Chip is NOT BGT60TR13C.")
            return RET_VAL_ERR

    def set_register_config_file(self, file_name):
        self.__register_config_file_name = file_name

    def __load_register_config_file(self):
        file_name = self.__register_config_file_name
        logging.info(f"Loading radar_config file: {file_name}")
        with open(file_name, 'r') as file:
            for line in file:
                line = line.strip()
                parts = line.split()
                if len(parts) == 3:
                    label, address_str, value_str = parts
                    address = int(address_str, 16)
                    value = int(value_str, 16)
                    if address == BGT60TRXX_REG_SFCTL:
                        if self.__spi.max_speed_hz>20_000_000:
                            value |= BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK
                        else:
                            value &= ~BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK
                    self.__set_reg(address, value)
                else:
                    raise RuntimeError(f"Line format is incorrect: {line}.")

    def start(self):
        if self.__data_collection_thread and self.__data_collection_thread.is_alive():
            self.stop()
        if self.__data_collection_thread is None or not self.__data_collection_thread.is_alive():
            self.__data_collection_stop_event.clear()
            self.__data_collection_thread = threading.Thread(target=self.__data_collection)
            self.__data_collection_thread.start()

    def stop(self):
        ret_temp = RET_VAL_OK
        if self.soft_reset(BGT60TRXX_RESET_SW) != RET_VAL_OK:
            ret_temp = RET_VAL_ERR
        if self.__data_collection_thread and self.__data_collection_thread.is_alive():
            self.__data_collection_stop_event.set()
            self.__data_collection_thread.join()
        if self.__file_fd is not None:
            self.__file_fd.close()
            self.__file_fd = None
        return ret_temp

    def soft_reset(self, reset_type):
        self.__sub_frame_buffer = []
        status = self.__get_reg(BGT60TRXX_REG_MAIN)
        tmp = status | reset_type
        self.__set_reg(BGT60TRXX_REG_MAIN, tmp)

        time_out_cnt = 0
        N = 10
        for i in range(N):
            time.sleep(0.01)
            status = self.__get_reg(BGT60TRXX_REG_MAIN)
            if status & reset_type == 0:
               break
            time_out_cnt += 1
        if time_out_cnt == N:
            RuntimeError("Soft reset timeout!")
        else:
            time.sleep(0.01)

    def hard_reset(self):
        self.__rst.on()
        time.sleep(0.01)
        self.__rst.off()
        time.sleep(0.01)
        self.__rst.on()
        time.sleep(0.01)

    def __check_fifo_status(self):
        status = self.__get_reg(BGT60TRXX_REG_FSTAT_TR13C)
        FOF_ERR = (status & BGT60TRXX_REG_FSTAT_FOF_ERR_MSK)>>BGT60TRXX_REG_FSTAT_FOF_ERR_POS
        FULL = (status & BGT60TRXX_REG_FSTAT_FULL_MSK)>>BGT60TRXX_REG_FSTAT_FULL_POS
        CREF = (status & BGT60TRXX_REG_FSTAT_CREF_MSK)>>BGT60TRXX_REG_FSTAT_CREF_POS
        EMPTY = (status & BGT60TRXX_REG_FSTAT_EMPTY_MSK)>>BGT60TRXX_REG_FSTAT_EMPTY_POS
        FUF_ERR = (status & BGT60TRXX_REG_FSTAT_FUF_ERR_MSK)>>BGT60TRXX_REG_FSTAT_FUF_ERR_POS
        SPI_BURST_ERR = (status & BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_MSK) >> BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_POS
        CLK_NUM_ERR = (status & BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_MSK)>>BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_POS
        FILL_STATUS = (status & BGT60TRXX_REG_FSTAT_FILL_STATUS_MSK)>>BGT60TRXX_REG_FSTAT_FILL_STATUS_POS
        return FOF_ERR, FULL, CREF, EMPTY, FUF_ERR, SPI_BURST_ERR, CLK_NUM_ERR, FILL_STATUS

    def __set_fifo_limit(self, num_samples):
        if 0 < num_samples>>1 <= BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE and num_samples % 2 == 0:
            tmp = self.__get_reg(BGT60TRXX_REG_SFCTL)
            tmp &= ~BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK
            # The FIFO is 24-bit wide, which is 2 12-bit ADC samples.
            tmp |= (((num_samples >> 1) - 1) << BGT60TRXX_REG_SFCTL_FIFO_CREF_POS) & BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK
            self.__set_reg(BGT60TRXX_REG_SFCTL, tmp)
            return RET_VAL_OK
        else:
            RuntimeError("Invalid num_samples.")
            return RET_VAL_ERR

    def print_fifo_status(self):
        FOF_ERR, FULL, CREF, EMPTY, FUF_ERR, SPI_BURST_ERR, CLK_NUM_ERR, FILL_STATUS = self.__check_fifo_status()
        logging.info(f"FOF_ERR: {FOF_ERR}, FUF_ERR: {FUF_ERR}, SPI_BURST_ERR:{SPI_BURST_ERR}, CLK_NUM_ERR: {CLK_NUM_ERR}")

    def print_gsr_reg(self):
        if self.__last_gsr_reg & BGT60TRXX_REG_GSR0_FOU_ERR_MSK:
            logging.info("GSR FIFO OVERFLOW/UNDERFLOW ERROR: 1")
        else:
            logging.info("GSR FIFO OVERFLOW/UNDERFLOW ERROR: 0")

        if self.__last_gsr_reg & BGT60TRXX_REG_GSR0_MISO_HS_READ_MSK:
            logging.info("GSR MISO HS: 1")
        else:
            logging.info("GSR MISO HS: 0")

        if self.__last_gsr_reg & BGT60TRXX_REG_GSR0_SPI_BURST_ERR_MSK:
            logging.info("GSR SPI BURST ERR: 1")
        else:
            logging.info("GSR SPI BURST ERR: 0")

        if self.__last_gsr_reg & BGT60TRXX_REG_GSR0_CLK_NUM_ERR_MSK:
            logging.info("GSR CLOCK NUMBER ERR: 1")
        else:
            logging.info("GSR CLOCK NUMBER ERR: 0")

    def check_gsr_reg(self):
        if self.__last_gsr_reg & (BGT60TRXX_REG_GSR0_FOU_ERR_MSK | BGT60TRXX_REG_GSR0_SPI_BURST_ERR_MSK | BGT60TRXX_REG_GSR0_CLK_NUM_ERR_MSK):
            return RET_VAL_ERR
        else:
            return RET_VAL_OK

    def __data_collection(self):
        logging.debug("Data collection thread started.")

        while not self.__data_collection_stop_event.is_set():
            # 1. Software reset
            self.soft_reset(BGT60TRXX_RESET_SW)

            # 2. Load register
            self.__load_register_config_file()

            # 3. Set fifo limit
            self.__set_fifo_parameters()

            # 4. Create file
            if self.__file_fd is not None:
                self.__file_fd.close()
                self.__file_fd = None
            if self.__save_to_file is not None:
                # Get the current timestamp at millisecond precision
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

                # Add the timestamp to the filename
                filename_with_timestamp = f"{self.__save_to_file}_{timestamp}.bin"

                # Ensure the directory exists
                directory = os.path.dirname(filename_with_timestamp)
                if directory:  # Check if a directory is specified in the path
                    os.makedirs(directory, exist_ok=True)

                self.__file_fd = open(filename_with_timestamp, "wb", buffering=0)

            # 5. Start frame
            self.__seq = 0
            status = self.__get_reg(BGT60TRXX_REG_MAIN)
            tmp = status | BGT60TRXX_REG_MAIN_FRAME_START_MSK
            self.__set_reg(BGT60TRXX_REG_MAIN, tmp)

            error_flag = False

            while not self.__data_collection_stop_event.is_set():
                time.sleep(0.001)
                # Error detected, we need a soft reset. Also, a new file should be created.
                if error_flag:
                    break

                # A loop is required to support small bursts.
                while True:
                    # Check the FIFO status reg instead of the IRQ pin, which is vulnerable to noise.
                    # This active check is less efficient, but more robust.
                    FOF_ERR, FULL, CREF, EMPTY, FUF_ERR, SPI_BURST_ERR, CLK_NUM_ERR, FILL_STATUS = self.__check_fifo_status()
                    # Significant errors detected, an exception is raised and a reset is required.
                    if FOF_ERR or FUF_ERR or SPI_BURST_ERR or CLK_NUM_ERR:
                        logging.error(f"FOF_ERR: {FOF_ERR}, FUF_ERR: {FUF_ERR}, SPI_BURST_ERR:{SPI_BURST_ERR}, CLK_NUM_ERR: {CLK_NUM_ERR}")
                        error_flag = True
                        break
                    # Data is not ready. Wait for the next timer.
                    if CREF != 1:
                        break

                    fifo_data = self.__get_fifo_data(self.__num_sampler_per_burst)
                    try:
                        self.__sub_frame_buffer += fifo_data
                        while len(self.__sub_frame_buffer) >= self.__num_bytes_per_frame:
                            frame = self.__sub_frame_buffer[:self.__num_bytes_per_frame]
                            self.__sub_frame_buffer = self.__sub_frame_buffer[self.__num_bytes_per_frame:]
                            full_frame = []
                            if self.__version == 0:
                                # version(4 -byte) seq(4-byte) length(4-byte) + data(length-byte)
                                full_frame = (self.__version.to_bytes(4, byteorder='little', signed=False) +
                                              self.__seq.to_bytes(4, byteorder='little', signed=False) +
                                              len(frame).to_bytes(4, byteorder='little', signed=False) +
                                              bytes(frame))
                                self.__seq += 1
                            if self.__file_fd is not None:
                                self.__file_fd.write(full_frame)
                            self.frame_buffer.put(full_frame, block=False)
                    except queue.Full:
                        logging.warning("Queue is full!")
        logging.debug("Data collection thread stopped.")

    def __del__(self):
        self.stop()

