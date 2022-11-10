import time
import struct
import collections as clc


__version__ = '0.0.2'

CHIP_ID = 0xEA
CHIP_ID_20649 = 0xE1


I2C_ADDR = 0x68
I2C_ADDR_ALT = 0x69
ICM20948_BANK_SEL = 0x7f

ICM20948_I2C_MST_ODR_CONFIG = 0x00
ICM20948_I2C_MST_CTRL = 0x01
ICM20948_I2C_MST_DELAY_CTRL = 0x02
ICM20948_I2C_SLV0_ADDR = 0x03
ICM20948_I2C_SLV0_REG = 0x04
ICM20948_I2C_SLV0_CTRL = 0x05
ICM20948_I2C_SLV0_DO = 0x06
ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B

ICM20948_GYRO_SMPLRT_DIV = 0x00
ICM20948_GYRO_CONFIG_1 = 0x01
ICM20948_GYRO_CONFIG_2 = 0x02

# Bank 0
ICM20948_WHO_AM_I = 0x00
ICM20948_USER_CTRL = 0x03
ICM20948_PWR_MGMT_1 = 0x06
ICM20948_PWR_MGMT_2 = 0x07
ICM20948_INT_PIN_CFG = 0x0F

ICM20948_ACCEL_SMPLRT_DIV_1 = 0x10
ICM20948_ACCEL_SMPLRT_DIV_2 = 0x11
ICM20948_ACCEL_INTEL_CTRL = 0x12
ICM20948_ACCEL_WOM_THR = 0x13
ICM20948_ACCEL_CONFIG = 0x14
ICM20948_ACCEL_XOUT_H = 0x2D
ICM20948_GRYO_XOUT_H = 0x33

ICM20948_TEMP_OUT_H = 0x39
ICM20948_TEMP_OUT_L = 0x3A

# Offset and sensitivity - defined in electrical characteristics, and TEMP_OUT_H/L of datasheet
ICM20948_TEMPERATURE_DEGREES_OFFSET = 21
ICM20948_TEMPERATURE_SENSITIVITY = 333.87
ICM20948_ROOM_TEMP_OFFSET = 21

AK09916_I2C_ADDR = 0x0c
AK09916_CHIP_ID = 0x09
AK09916_WIA = 0x01
AK09916_ST1 = 0x10
AK09916_ST1_DOR = 0b00000010   # Data overflow bit
AK09916_ST1_DRDY = 0b00000001  # Data self.ready bit
AK09916_HXL = 0x11
AK09916_ST2 = 0x18
AK09916_ST2_HOFL = 0b00001000  # Magnetic sensor overflow bit
AK09916_CNTL2 = 0x31
AK09916_CNTL2_MODE = 0b00001111
AK09916_CNTL2_MODE_OFF = 0
AK09916_CNTL2_MODE_SINGLE = 1
AK09916_CNTL2_MODE_CONT1 = 2
AK09916_CNTL2_MODE_CONT2 = 4
AK09916_CNTL2_MODE_CONT3 = 6
AK09916_CNTL2_MODE_CONT4 = 8
AK09916_CNTL2_MODE_TEST = 16
AK09916_CNTL3 = 0x32

DEG_2_RAD = 0.017453293  # Degrees/s to rad/s multiplier

G_TO_ACCEL = 9.80665


class ICM20x:
    scalers=clc.defaultdict(dict)
    scalers['ICM20649']=clc.defaultdict(dict)
    scalers['ICM20948']=clc.defaultdict(dict)

    scalers['ICM20649']['accel']['scale2byte']={4:0b00,8:0b01,16:0b10,30:0b11}
    scalers['ICM20649']['accel']['scale2scaler']={4:8192,8:4096.0,16:2048,30:1024}
    scalers['ICM20649']['accel']['idx2scaler']={0:8192,1:4096.0,2:2048,3:1024}
    
    scalers['ICM20649']['gyro']['scale2byte']={500:0b00,1000:0b01,2000:0b10,4000:0b11}
    scalers['ICM20649']['gyro']['scale2scaler']={500:65.5,1000:32.8,2000:16.4,4000:8.2}
    scalers['ICM20649']['gyro']['idx2scaler']={0:65.5,1:32.8,2:16.4,3:8.2}



    scalers['ICM20948']['accel']['scale2byte']={2:0b00,4:0b01,8:0b10,16:0b11}
    scalers['ICM20948']['accel']['scale2scaler']={2:16384,4:8192,8:4096.0,16:2048}
    scalers['ICM20948']['accel']['idx2scaler']={0:16384,1:8192,2:4096.0,3:2048}

    scalers['ICM20948']['gyro']['scale2byte']={250:0b00,500:0b01,1000:0b10,2000:0b11}
    scalers['ICM20948']['gyro']['scale2scaler']={250:131.0,500:65.5,1000:32.8,2000:16.4}
    scalers['ICM20948']['gyro']['idx2scaler']={0:131.0,1:65.5,2:32.8,3:16.4}

    _accelscale_cache=None
    _gyroscale_cache=None

    def write(self, reg, value):
        """Write byte to the sensor."""
        self._bus.write_byte_data(self._addr, reg, value)
        time.sleep(0.0001)

    def read(self, reg):
        """Read byte from the sensor."""
        return self._bus.read_byte_data(self._addr, reg)

    def trigger_mag_io(self):
        user = self.read(ICM20948_USER_CTRL)
        self.write(ICM20948_USER_CTRL, user | 0x20)
        time.sleep(0.005)
        self.write(ICM20948_USER_CTRL, user)

    def read_bytes(self, reg, length=1):
        """Read byte(s) from the sensor."""
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    def bank(self, value):
        """Switch register self.bank."""
        if not self._bank == value:
            self.write(ICM20948_BANK_SEL, value << 4)
            self._bank = value

    def mag_write(self, reg, value):
        """Write a byte to the slave magnetometer."""
        self.bank(3)
        self.write(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR)  # Write one byte
        self.write(ICM20948_I2C_SLV0_REG, reg)
        self.write(ICM20948_I2C_SLV0_DO, value)
        self.bank(0)
        self.trigger_mag_io()

    def mag_read(self, reg):
        """Read a byte from the slave magnetometer."""
        self.bank(3)
        self.write(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80)
        self.write(ICM20948_I2C_SLV0_REG, reg)
        self.write(ICM20948_I2C_SLV0_DO, 0xff)
        self.write(ICM20948_I2C_SLV0_CTRL, 0x80 | 1)  # Read 1 byte

        self.bank(0)
        self.trigger_mag_io()

        return self.read(ICM20948_EXT_SLV_SENS_DATA_00)

    def mag_read_bytes(self, reg, length=1):
        """Read up to 24 bytes from the slave magnetometer."""
        self.bank(3)
        self.write(ICM20948_I2C_SLV0_CTRL, 0x80 | 0x08 | length)
        self.write(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80)
        self.write(ICM20948_I2C_SLV0_REG, reg)
        self.write(ICM20948_I2C_SLV0_DO, 0xff)
        self.bank(0)
        self.trigger_mag_io()

        return self.read_bytes(ICM20948_EXT_SLV_SENS_DATA_00, length)

    def magnetometer_ready(self):
        """Check the magnetometer status self.ready bit."""
        return self.mag_read(AK09916_ST1) & 0x01 > 0

    def read_magnetometer_data(self, timeout=1.0):
        self.mag_write(AK09916_CNTL2, 0x01)  # Trigger single measurement
        t_start = time.time()
        while not self.magnetometer_ready():
            if time.time() - t_start > timeout:
                raise RuntimeError("Timeout waiting for Magnetometer Ready")
            time.sleep(0.00001)

        data = self.mag_read_bytes(AK09916_HXL, 6)

        # Read ST2 to confirm self.read finished,
        # needed for continuous modes
        # self.mag_read(AK09916_ST2)

        x, y, z = struct.unpack("<hhh", bytearray(data))

        # Scale for magnetic flux density "uT"
        # from section 3.3 of the datasheet
        # This value is constant
        x *= 0.15
        y *= 0.15
        z *= 0.15

        return x, y, z

    def read_accelerometer_gyro_data(self):
        self.bank(0)
        data = self._bus.read_i2c_block_data(self._addr, ICM20948_ACCEL_XOUT_H, 6)
        # data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        ax, ay, az = struct.unpack(">hhh", bytearray(data))

        # self.bank(2)
        # Read accelerometer full scale range and
        # use it to compensate the self.reading to gs
        # scale = (self.read(ICM20948_ACCEL_CONFIG) & 0x06) >> 1

        # scale ranges from section 3.2 of the datasheet
        # gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]

        gs = self.scalers[self.imutype]['accel']['scale2scaler'][self._accelscale_cache]
        ax /= gs
        ay /= gs
        az /= gs

        ax *= G_TO_ACCEL
        ay *= G_TO_ACCEL
        az *= G_TO_ACCEL

        # Read back the degrees per second rate and
        # use it to compensate the self.reading to dps
        # scale = (self.read(ICM20948_GYRO_CONFIG_1) & 0x06) >> 1

        # scale ranges from section 3.1 of the datasheet
        # dps = [131, 65.5, 32.8, 16.4][scale]
        self.bank(0)
        data = self._bus.read_i2c_block_data(self._addr, ICM20948_GRYO_XOUT_H, 6)
        # data = self.read_bytes(ICM20948_ACCEL_XOUT_H, 12)

        gx, gy, gz = struct.unpack(">hhh", bytearray(data))

        dps = self.scalers[self.imutype]['gyro']['scale2scaler'][self._gyroscale_cache]

        gx /= dps
        gy /= dps
        gz /= dps

        gx *= DEG_2_RAD
        gy *= DEG_2_RAD
        gz *= DEG_2_RAD

        return ax, ay, az, gx, gy, gz

    def set_accelerometer_sample_rate(self, rate=125):
        """Set the accelerometer sample rate in Hz."""
        self.bank(2)
        # 125Hz - 1.125 kHz / (1 + rate)
        rate = int((1125.0 / rate) - 1)
        # TODO maybe use struct to pack and then write_bytes
        self.write(ICM20948_ACCEL_SMPLRT_DIV_1, (rate >> 8) & 0xff)
        self.write(ICM20948_ACCEL_SMPLRT_DIV_2, rate & 0xff)

    def set_accelerometer_full_scale(self, scale=16):
        # scale = 2,4,8,16
        """Set the accelerometer fulls cale range to +- the supplied value."""
        self.bank(2)
        value = self.read(ICM20948_ACCEL_CONFIG) & 0b11111001
        bt = self.scalers[self.imutype]['accel']['scale2byte'][scale]
        # value |= {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}[scale] << 1
        value |= bt << 1
        self.write(ICM20948_ACCEL_CONFIG, value)

        self._accelscale_cache = scale

    def set_accelerometer_low_pass(self, enabled=True, mode=5):
        """Configure the accelerometer low pass filter."""
        self.bank(2)
        value = self.read(ICM20948_ACCEL_CONFIG) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(ICM20948_ACCEL_CONFIG, value)

    def set_gyro_sample_rate(self, rate=100):
        """Set the gyro sample rate in Hz."""
        self.bank(2)
        # 100Hz sample rate - 1.1 kHz / (1 + rate)
        rate = int((1100.0 / rate) - 1)
        self.write(ICM20948_GYRO_SMPLRT_DIV, rate)

    def set_gyro_full_scale(self, scale=250):
        # scale = 250 500 1000 2000
        """Set the gyro full scale range to +- supplied value."""
        self.bank(2)
        value = self.read(ICM20948_GYRO_CONFIG_1) & 0b11111001
        bt = self.scalers[self.imutype]['gyro']['scale2byte'][scale]
        value |= bt << 1
        # value |= {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}[scale] << 1
        self.write(ICM20948_GYRO_CONFIG_1, value)

        self._gyroscale_cache = scale

    def set_gyro_low_pass(self, enabled=True, mode=5):
        """Configure the gyro low pass filter."""
        self.bank(2)
        value = self.read(ICM20948_GYRO_CONFIG_1) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(ICM20948_GYRO_CONFIG_1, value)

    def read_temperature(self):
        """Property to read the current IMU temperature"""
        # PWR_MGMT_1 defaults to leave temperature enabled
        self.bank(0)
        temp_raw_bytes = self.read_bytes(ICM20948_TEMP_OUT_H, 2)
        temp_raw = struct.unpack('>h', bytearray(temp_raw_bytes))[0]
        temperature_deg_c = ((temp_raw - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_TEMPERATURE_SENSITIVITY) + ICM20948_TEMPERATURE_DEGREES_OFFSET
        return temperature_deg_c

    def __init__(self, i2c_addr=I2C_ADDR, i2c_bus=None,imutype='ICM20649'):
        self.imutype = imutype 
               
        self._bank = -1
        self._addr = i2c_addr

        self._bus = i2c_bus

        self.bank(0)
        physical_chipID = self.read(ICM20948_WHO_AM_I)
        if not (physical_chipID == CHIP_ID or physical_chipID == CHIP_ID_20649):
            raise RuntimeError("Unable to find ICM20948 or ICM20649")

        self.write(ICM20948_PWR_MGMT_1, 0x80)
        time.sleep(0.01)
        self.write(ICM20948_PWR_MGMT_1, 0x01)
        self.write(ICM20948_PWR_MGMT_2, 0x00)

        self.bank(2)

        self.set_gyro_sample_rate(150)
        self.set_gyro_low_pass(enabled=False, mode=5)
        self.set_gyro_full_scale(500)

        self.set_accelerometer_sample_rate(150)
        self.set_accelerometer_low_pass(enabled=False, mode=5)
        self.set_accelerometer_full_scale(8)

        self.bank(0)
        self.write(ICM20948_INT_PIN_CFG, 0x30)

        self.bank(3)
        self.write(ICM20948_I2C_MST_CTRL, 0x4D)
        self.write(ICM20948_I2C_MST_DELAY_CTRL, 0x01)

        if self.imutype == 'ICM20948':
            if not self.mag_read(AK09916_WIA) == AK09916_CHIP_ID:
                raise RuntimeError("Unable to find AK09916")

            # Reset the magnetometer
            self.mag_write(AK09916_CNTL3, 0x01)
            while self.mag_read(AK09916_CNTL3) == 0x01:
                time.sleep(0.0001)


if __name__ == "__main__":
    from smbus import SMBus
    i2c_bus = SMBus(1)

    imu = ICM20x(i2c_addr=0x68,i2c_bus=i2c_bus,imutype='ICM20649')

    while True:
        # x, y, z = imu.read_magnetometer_data()
        try:
            ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
            print("""
                Accel: {:05.5f} {:05.5f} {:05.5f}
                Gyro:  {:05.5f} {:05.5f} {:05.5f}""".format(
                            ax, ay, az, gx, gy, gz,
                        ))


        except:
            print("Error read")

        time.sleep(0.015)
