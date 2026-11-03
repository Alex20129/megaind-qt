#include "megaind/megaind_device.hpp"

#include <QDebug>
#include <QThread>
#include <QTimer>
#include <cmath>
#include <cstring>
#include <unistd.h>

extern "C"
{
	#include "megaind.h"
	#include "comm.h"
	int doBoardInit(int stack);
	int boardCheck(int stack);
	int setUinType(int dev, u8 ch, u8 val);
	int optoChGet(int dev, u8 channel, OutStateEnumType* state);
	int optoGet(int dev, int* val);
	int optoCountGet(int dev, u8 ch, u16* val);
	int optoCountReset(int dev, u8 ch);
	int optoEdgeGet(int dev, u8 ch, u8* val);
	int optoEdgeSet(int dev, u8 ch, u8 val);
	int rs485Set(int dev, u8 mode, u32 baud, u8 stopB, u8 parity, u8 add);
	int rs485Get(int dev);
}

namespace MegaInd
{

MegaIndDevice::MegaIndDevice(BoardId board_id, QObject *parent) : QObject(parent)
{
	optoInputsPollingTimer=new QTimer(this);
	analogInputsPollingTimer=new QTimer(this);
	oneWireDevicesPollingTimer=new QTimer(this);
	countersPollingTimer=new QTimer(this);

	connect(optoInputsPollingTimer, &QTimer::timeout, this, &MegaIndDevice::processOptoInputs);
	connect(analogInputsPollingTimer, &QTimer::timeout, this, &MegaIndDevice::processAnalogInputs);
	connect(oneWireDevicesPollingTimer, &QTimer::timeout, this, &MegaIndDevice::processOneWireDevices);
	connect(countersPollingTimer, &QTimer::timeout, this, &MegaIndDevice::processCounters);
}

MegaIndDevice::~MegaIndDevice()
{
	close();
}

bool MegaIndDevice::open()
{
	QMutexLocker locker(&mutex);
	if (fd>0)
	{
		return true; // Already open
	}
	fd=doBoardInit(static_cast<int>(mBoardId));
	if (fd>0)
	{
		emit connectionChanged(true);
		return true;
	}
	emit errorOccurred(QString("Failed to open board %1").arg(mBoardId));
	return false;
}

void MegaIndDevice::close()
{
	QMutexLocker locker(&mutex);
	if (fd>0)
	{
		::close(fd);
		fd=-1;
		emit connectionChanged(false);
	}
}

bool MegaIndDevice::isConnected() const
{
	return (fd>0);
}

MegaIndDevice::BoardId MegaIndDevice::boardId() const
{
	return mBoardId;
}

MegaIndDevice::BoardInfo MegaIndDevice::getBoardInfo() const
{
	uint8_t buff[5];
	BoardInfo info;
	if (!isValid()) return info;
	QMutexLocker locker(&mutex);
	int resp=i2cMem8Read(fd, I2C_MEM_DIAG_TEMPERATURE, buff, 5);
	if (resp == OK)
	{
		info.temperature=buff[0];
		int16_t val;
		memcpy(&val, &buff[1], 2);
		info.powerVoltage=val / VOLT_SCALE;
		memcpy(&val, &buff[3], 2);
		info.raspberryVoltage=val / VOLT_SCALE;
	}
	// Get firmware version
	resp=i2cMem8Read(fd, I2C_MEM_REVISION_MAJOR, buff, 2);
	if (resp == OK)
	{
		info.firmwareMajor=buff[0];
		info.firmwareMinor=buff[1];
	}
	// Get battery voltage
	resp=i2cMem8Read(fd, I2C_MEM_DIAG_3V, buff, 2);
	if (resp == OK)
	{
		int16_t val;
		memcpy(&val, buff, 2);
		info.batteryVoltage=val / VOLT_SCALE;
	}
	return info;
}

// Analog outputs
bool MegaIndDevice::setVoltageOut(Channel ch, float voltage)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);
	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}
	if (voltage<0.0)
	{
		voltage=0.0;
	}
	if (voltage>10.0)
	{
		voltage=10.0;
	}
	return val16Set(fd, I2C_MEM_U0_10_OUT_VAL1, ch, VOLT_SCALE, voltage) == OK;
}

float MegaIndDevice::getVoltageOut(Channel ch) const
{
	if (!isValid())
	{
		return -1.0f;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return -1.0f;
	}
	float val=0;
	if (val16Get(fd, I2C_MEM_U0_10_OUT_VAL1, ch, &val) == OK)
	{
		return val/VOLT_SCALE;
	}
	return -1.0f;
}

bool MegaIndDevice::setCurrentOut(Channel ch, float current)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);
	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}
	if (current<4.0)
	{
		current=4.0;
	}
	else if (current>20.0)
	{
		current=20.0;
	}
	return val16Set(fd, I2C_MEM_I4_20_OUT_VAL1, ch, CURRENT_SCALE, current) == OK;
}

float MegaIndDevice::getCurrentOut(Channel ch) const
{
	if (!isValid())
	{
		return -1.0f;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return -1.0f;
	}
	float val=0;
	if (val16Get(fd, I2C_MEM_I4_20_OUT_VAL1, ch, &val) == OK)
	{
		return val/CURRENT_SCALE;
	}
	return -1.0f;
}

bool MegaIndDevice::setOpenDrainPwmDuty(Channel ch, uint16_t duty)
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}
	if (duty>OpenDrainDutyCycleMax)
	{
		duty=OpenDrainDutyCycleMax;
	}
	uint8_t buff[2];
	uint8_t add=I2C_MEM_OD_PWM1 + 2 * (ch - 1);
	memcpy(buff, &duty, 2);
	return i2cMem8Write(fd, add, buff, 2) == OK;
}

uint16_t MegaIndDevice::getOpenDrainPwmDuty(Channel ch) const
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return 0;
	}
	if (ch<Channel1 || ch>Channel4)
	{
		return 0;
	}
	uint8_t buff[2];
	uint8_t add=I2C_MEM_OD_PWM1 + 2 * (ch - 1);
	if (i2cMem8Read(fd, add, buff, 2) == OK)
	{
		uint16_t val;
		memcpy(&val, buff, 2);
		return val;
	}
	return 0;
}

bool MegaIndDevice::setOpenDrainPwmFrequency(Channel ch, uint16_t freq)
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}
	if (ch<Channel1 || ch>Channel3)
	{
		return false;
	}
	if ((freq<10) || (freq>6400))
	{
		return false;
	}
	float fFreq=static_cast<float>(freq);
	if (val16Set(fd, I2C_MEM_OD1_FREQ, ch, 1, fFreq) != OK)
	{
		emit errorOccurred("Failed to set open drain PWM frequency");
		return false;
	}
	return true;
}

uint16_t MegaIndDevice::getOpenDrainPwmFrequency(Channel ch) const
{
	if (!isValid())
	{
		return 0;
	}
	if (ch<Channel1 || ch>Channel3)
	{
		return 0;
	}
	float val=0;
	if (val16Get(fd, I2C_MEM_OD1_FREQ, ch, &val) == OK)
	{
		return static_cast<uint16_t>(val);
	}
	return 0;
}

// Analog inputs
float MegaIndDevice::getVoltageIn(Channel ch) const
{
	float val1=-1.25;
	if (!isValid())
	{
		return val1;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return val1;
	}
	if (val16Get(fd, I2C_MEM_U0_10_IN_VAL1, ch, &val1) != OK)
	{
		return val1;
	}
	else
	{
		return val1/VOLT_SCALE;
	}
}

float MegaIndDevice::getVoltageInPM(Channel ch) const
{
	float val1=-10.25;
	if (!isValid())
	{
		return val1;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return val1;
	}
	if (val16Get(fd, I2C_MEM_U_PM_10_IN_VAL1, ch, &val1) != OK)
	{
		return val1;
	}
	return val1/VOLT_SCALE;
}

float MegaIndDevice::getCurrentIn(Channel ch) const
{
	float val1=-1.25;
	if (!isValid())
	{
		return val1;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return val1;
	}
	if (val16Get(fd, I2C_MEM_I4_20_IN_VAL1, ch, &val1) != OK)
	{
		return val1;
	}
	return val1/CURRENT_SCALE;
}

// Digital inputs
bool MegaIndDevice::getOptoInput(Channel ch) const
{
	OutStateEnumType state=OFF;
	if (!isValid())
	{
		return false;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}
	QMutexLocker locker(&mutex);
	if (optoChGet(fd, ch, &state) == OK)
	{
		return state == ON;
	}
	return false;
}

uint8_t MegaIndDevice::getOptoInputs() const
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return 0;
	}
	int val=0;
	if (optoGet(fd, &val) == OK)
	{
		return static_cast<uint8_t>(val & 0x0F);
	}
	return 0;
}

uint32_t MegaIndDevice::getOptoCounter(Channel ch) const
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return 0;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return 0;
	}
	uint16_t val=0;
	if (optoCountGet(fd, ch, &val) == OK)
	{
		return val;
	}
	return 0;
}

bool MegaIndDevice::resetOptoCounter(Channel ch)
{
	QMutexLocker locker(&mutex);
	if (!isValid() || ch<Channel1 || ch>Channel4)
	{
		return false;
	}
	return (optoCountReset(fd, ch) == OK);
}

bool MegaIndDevice::setOptoEdgeDetection(Channel ch, bool rising, bool falling)
{
	QMutexLocker locker(&mutex);
	if (!isValid() || ch<Channel1 || ch>Channel4)
	{
		return false;
	}
	uint8_t buff[1];
	uint8_t mask=1 << (ch - 1);
	if (i2cMem8Read(fd, I2C_MEM_OPTO_RISING_ENABLE, buff, 1) != OK)
	{
		return false;
	}
	if (rising)
	{
		buff[0] |= mask;
	}
	else
	{
		buff[0] &= ~mask;
	}
	if (i2cMem8Write(fd, I2C_MEM_OPTO_RISING_ENABLE, buff, 1) != OK)
	{
		return false;
	}
	if (i2cMem8Read(fd, I2C_MEM_OPTO_FALLING_ENABLE, buff, 1) != OK)
	{
		return false;
	}
	if (falling)
	{
		buff[0] |= mask;
	}
	else
	{
		buff[0] &= ~mask;
	}
	return (i2cMem8Write(fd, I2C_MEM_OPTO_FALLING_ENABLE, buff, 1) == OK);
}

MegaIndDevice::EdgeDetection MegaIndDevice::getOptoEdgeDetection(Channel ch) const
{
	QMutexLocker locker(&mutex);
	if (!isValid() || ch<Channel1 || ch>Channel4)
	{
		return EdgeNone;
	}
	uint8_t buff[1];
	uint8_t mask=1 << (ch - 1);
	bool rising=false;
	bool falling=false;
	if (i2cMem8Read(fd, I2C_MEM_OPTO_RISING_ENABLE, buff, 1) == OK)
	{
		rising=(buff[0] & mask) != 0;
	}
	if (i2cMem8Read(fd, I2C_MEM_OPTO_FALLING_ENABLE, buff, 1) == OK)
	{
		falling=(buff[0] & mask) != 0;
	}
	if (rising && falling)
	{
		return EdgeBoth;
	}
	if (rising)
	{
		return EdgeRising;
	}
	if (falling)
	{
		return EdgeFalling;
	}
	return EdgeNone;
}

float MegaIndDevice::getOptoFrequency(Channel ch) const
{
	float val=-1.25;
	if (!isValid())
	{
		return val;
	}
	if ((ch<Channel1) || (ch>Channel4))
	{
		return val;
	}
	if (val16Get(fd, I2C_MEM_OPTO_FREQ1, ch, &val) != OK)
	{
		// ?
	}
	return val;
}

// Digital outputs
bool MegaIndDevice::setDigitalOut(Channel ch, bool state)
{
	QMutexLocker locker(&mutex);
	if (!isValid() || ch<Channel1 || ch>Channel4)
	{
		return false;
	}
	uint8_t buff[2];
	if (i2cMem8Read(fd, I2C_MEM_RELAY_VAL, buff, 1) != OK)
	{
		return false;
	}
	if (state)
	{
		buff[0] |= (1 << (ch - 1));
	}
	else
	{
		buff[0] &= ~(1 << (ch - 1));
	}
	return (i2cMem8Write(fd, I2C_MEM_RELAY_SET, buff, 1) == OK);
}

bool MegaIndDevice::getDigitalOut(Channel ch) const
{
	QMutexLocker locker(&mutex);
	if (!isValid() || ch<Channel1 || ch>Channel4)
	{
		return false;
	}
	uint8_t buff[1];
	if (i2cMem8Read(fd, I2C_MEM_RELAY_VAL, buff, 1) == OK)
	{
		return (buff[0] & (1 << (ch - 1))) != 0;
	}
	return false;
}

bool MegaIndDevice::setLed(Channel ch, bool state)
{
	QMutexLocker locker(&mutex);
	if (!isValid() || ch<Channel1 || ch>Channel4)
	{
		return false;
	}
	uint8_t buff[2];
	if (i2cMem8Read(fd, I2C_MEM_RELAY_VAL, buff, 1) != OK)
	{
		return false;
	}
	// LEDs are in upper 4 bits
	if (state) {
		buff[0] |= (1 << (ch + 3));  // LED bits start at bit 4
	} else {
		buff[0] &= ~(1 << (ch + 3));
	}
	return (i2cMem8Write(fd, I2C_MEM_RELAY_SET, buff, 1) == OK);
}

// 1-Wire functions
int MegaIndDevice::oneWireSensorCount()
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return 0;
	}
	uint8_t buff[1];
	if (i2cMem8Read(fd, I2C_MEM_1WB_DEV, buff, 1) == OK)
	{
		return static_cast<int>(buff[0]);
	}
	return 0;
}

bool MegaIndDevice::oneWireScan()
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}
	uint8_t buff[1]={0xAA};
	return i2cMem8Write(fd, I2C_MEM_1WB_START_SEARCH, buff, 1) == OK;
}

float MegaIndDevice::getOneWireTemp(int index) const
{
	QMutexLocker locker(&mutex);
	if (!isValid() || (index<0) || (index>15))
	{
		return -999.0f;
	}

	uint8_t buff[2];
	int addr=I2C_MEM_1WB_T1 + index * OWB_TEMP_SIZE_B;

	if (i2cMem8Read(fd, addr, buff, 2) == OK)
	{
		int16_t raw;
		memcpy(&raw, buff, 2);
		float T=raw / 100.0f;
		return T;
	}
	return -999.0f;
}

uint64_t MegaIndDevice::getOneWireRomId(int index) const
{
	QMutexLocker locker(&mutex);
	if (!isValid() || (index<0) || (index>15))
	{
		return 0;
	}
	uint8_t buff[8];

	buff[0]=static_cast<uint8_t>(index & 0xFF);
	if (i2cMem8Write(fd, I2C_MEM_1WB_ROM_CODE_IDX, buff, 1) != OK) {
		qWarning() << "Failed to write sensor index" << index;
		return 0;
	}

	QThread::msleep(10);

	// Check the number of connected sensors
	if (i2cMem8Read(fd, I2C_MEM_1WB_DEV, buff, 1) != OK) {
		qWarning() << "Failed to read sensor count";
		return 0;
	}

	int sensorCount=buff[0];
	if (index >= sensorCount)
	{
		qWarning() << "Index" << index << "out of range, only" << sensorCount << "sensors";
		return 0;
	}

	// Read ROM ID
	if (i2cMem8Read(fd, I2C_MEM_1WB_ROM_CODE, buff, 8) == OK)
	{
		uint64_t id;
		memcpy(&id, buff, 8);
		return id;
	}

	qWarning() << "Failed to read ROM code for sensor" << index;
	return 0;
}

// RTC functions
QDateTime MegaIndDevice::getRtcTime() const
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return QDateTime();
	}
	uint8_t buff[6];
	if (i2cMem8Read(fd, I2C_RTC_YEAR_ADD, buff, 6) == OK)
	{
		return QDateTime(
			QDate(2000 + buff[0], buff[1], buff[2]),
			QTime(buff[3], buff[4], buff[5])
		);
	}
	return QDateTime();
}

bool MegaIndDevice::setRtcTime(const QDateTime &time)
{
	QMutexLocker locker(&mutex);
	if (!isValid() || !time.isValid())
	{
		return false;
	}

	uint8_t buff[7];
	QDate date=time.date();
	QTime t=time.time();

	buff[0]=date.year() - 2000;
	buff[1]=date.month();
	buff[2]=date.day();
	buff[3]=t.hour();
	buff[4]=t.minute();
	buff[5]=t.second();

	if (i2cMem8Write(fd, I2C_RTC_SET_YEAR_ADD, buff, 6) != OK)
	{
		return false;
	}

	// Trigger RTC set
	buff[0]=0xAA;
	return i2cMem8Write(fd, I2C_RTC_CMD_ADD, buff, 1) == OK;

}

// Watchdog functions
bool MegaIndDevice::reloadWatchdog()
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}

		uint8_t buff[1]={WDT_RESET_SIGNATURE};
		return i2cMem8Write(fd, I2C_MEM_WDT_RESET_ADD, buff, 1) == OK;

}

bool MegaIndDevice::setWatchdogPeriod(uint16_t seconds)
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}

		uint8_t buff[2];
		memcpy(buff, &seconds, 2);
		return i2cMem8Write(fd, I2C_MEM_WDT_INTERVAL_SET_ADD, buff, 2) == OK;

}

uint16_t MegaIndDevice::getWatchdogPeriod() const
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return 0;
	}

		uint8_t buff[2];
		if (i2cMem8Read(fd, I2C_MEM_WDT_INTERVAL_GET_ADD, buff, 2) == OK) {
			uint16_t val;
			memcpy(&val, buff, 2);
			return val;
		}
		return 0;

}

uint16_t MegaIndDevice::getWatchdogResetCount() const
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return 0;
	}

		uint8_t buff[2];
		if (i2cMem8Read(fd, I2C_MEM_WDT_RESET_COUNT_ADD, buff, 2) == OK) {
			uint16_t val;
			memcpy(&val, buff, 2);
			return val;
		}
		return 0;

}

bool MegaIndDevice::clearWatchdogResetCount()
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}

		uint8_t buff[1]={0xAA};  // Clear signature
		return i2cMem8Write(fd, I2C_MEM_WDT_CLEAR_RESET_COUNT_ADD, buff, 1) == OK;

}

// RS485/Modbus implementation
MegaIndDevice::RS485Mode MegaIndDevice::getRS485Mode() const
{
	QMutexLocker locker(&mutex);
		if (!isValid()) return RS485_DISABLED;

		uint8_t buff[5];
		if (i2cMem8Read(fd, I2C_MODBUS_SETINGS_ADD, buff, 5) != OK) {
			return RS485_DISABLED;
		}

		ModbusSetingsType settings;
		memcpy(&settings, buff, sizeof(ModbusSetingsType));

		if (settings.mbType == 1)
		{
			return RS485_MODBUS_SLAVE;
		}

		if (settings.mbType == 0)
		{
			if (settings.mbBaud == 0 && settings.mbParity == 0 &&
				settings.mbStopB == 0 && settings.add == 0) {
				return RS485_BRIDGE;
			}
		}

		return RS485_DISABLED;

}

bool MegaIndDevice::setRS485Mode(RS485Mode mode)
{
	QMutexLocker locker(&mutex);
	if (!isValid())
	{
		return false;
	}

	switch(mode)
	{
		case RS485_DISABLED:
			if (rs485Set(fd, 0, 9600, 1, 0, 1) != OK) {
				emit errorOccurred("Failed to disable RS485");
				return false;
			}
			return true;

		case RS485_BRIDGE:
			if (rs485Set(fd, 0, 0, 0, 0, 0) != OK) {
				emit errorOccurred("Failed to set RS485 bridge mode");
				return false;
			}
			return true;

		case RS485_MODBUS_SLAVE:
			emit errorOccurred("Use configureModbusSlave() to enable Modbus mode");
			return false;
	}

	return false;
}

bool MegaIndDevice::configureModbusSlave(uint32_t baudRate, uint8_t slaveAddress,
										 uint8_t parity, uint8_t stopBits)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

	if (baudRate<1200 || baudRate>920600)
	{
		emit errorOccurred(QString("Invalid baudrate %1, must be [1200, 920600]").arg(baudRate));
		return false;
	}

	if (slaveAddress<1 || slaveAddress>255) {
		emit errorOccurred(QString("Invalid slave address %1, must be [1, 255]").arg(slaveAddress));
		return false;
	}

	if (stopBits<1 || stopBits>2)
	{
		emit errorOccurred(QString("Invalid stop bits %1, must be 1 or 2").arg(stopBits));
		return false;
	}

	if (parity>2)
	{
		emit errorOccurred(QString("Invalid parity %1, must be 0=none, 1=even, 2=odd").arg(parity));
		return false;
	}

	if (rs485Set(fd, 1, baudRate, stopBits, parity, slaveAddress) != OK)
	{
		emit errorOccurred("Failed to configure Modbus slave");
		return false;
	}

	return true;

}

// Calibration implementation
bool MegaIndDevice::calibrateVoltageIn(Channel ch, float voltage)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

		if ((ch<Channel1) || (ch>Channel4)) return false;
		if (voltage<0 || voltage>10) return false;

		uint8_t buff[4];
		uint16_t raw=static_cast<uint16_t>(std::ceil(voltage * VOLT_SCALE));

		memcpy(buff, &raw, 2);
		buff[2]=ch + CAL_0_10V_IN_START_ID - 1;
		buff[3]=CALIBRATION_KEY;

		if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
			emit errorOccurred("Failed to write voltage input calibration data");
			return false;
		}

		QThread::msleep(100); // Wait for calibration to process
		return true;

}

bool MegaIndDevice::calibrateCurrentIn(Channel ch, float current)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

		if ((ch<Channel1) || (ch>Channel4)) return false;
		if (current<0 || current>30) return false; // Extended range for calibration

		uint8_t buff[4];
		uint16_t raw=static_cast<uint16_t>(std::ceil(current * CURRENT_SCALE));

		memcpy(buff, &raw, 2);
		buff[2]=ch + CAL_4_20mA_IN_START_ID - 1;
		buff[3]=CALIBRATION_KEY;

		if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
			emit errorOccurred("Failed to write current input calibration data");
			return false;
		}

		QThread::msleep(100);
		return true;

}

bool MegaIndDevice::resetVoltageInCalibration(Channel ch)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

		if ((ch<Channel1) || (ch>Channel4)) return false;

		uint8_t buff[4]={0, 0, 0, 0};
		buff[2]=ch + CAL_0_10V_IN_START_ID - 1;
		buff[3]=RESET_CALIBRATION_KEY;

		if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
			emit errorOccurred("Failed to reset voltage input calibration");
			return false;
		}

		QThread::msleep(100);
		return true;

}

bool MegaIndDevice::resetCurrentInCalibration(Channel ch)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

		if ((ch<Channel1) || (ch>Channel4)) return false;

		uint8_t buff[4]={0, 0, 0, 0};
		buff[2]=ch + CAL_4_20mA_IN_START_ID - 1;
		buff[3]=RESET_CALIBRATION_KEY;

		if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
			emit errorOccurred("Failed to reset current input calibration");
			return false;
		}

		QThread::msleep(100);
		return true;

}

bool MegaIndDevice::calibrateVoltageOut(Channel ch, float voltage)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}
	if (voltage<0 || voltage>10) return false;

	uint8_t buff[4];
	uint16_t raw=static_cast<uint16_t>(std::ceil(voltage * VOLT_SCALE));

	memcpy(buff, &raw, 2);
	buff[2]=ch + CAL_0_10V_OUT_START_ID - 1;
	buff[3]=CALIBRATION_KEY;

	if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
		emit errorOccurred("Failed to write voltage output calibration data");
		return false;
	}

	QThread::msleep(100);
	return true;

}

bool MegaIndDevice::calibrateCurrentOut(Channel ch, float current)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}
	if (current<0 || current>30) return false;

	uint8_t buff[4];
	uint16_t raw=static_cast<uint16_t>(std::ceil(current * CURRENT_SCALE));

	memcpy(buff, &raw, 2);
	buff[2]=ch + CAL_4_20mA_OUT_START_ID - 1;
	buff[3]=CALIBRATION_KEY;

	if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
		emit errorOccurred("Failed to write current output calibration data");
		return false;
	}

	QThread::msleep(100);
	return true;
}

bool MegaIndDevice::resetVoltageOutCalibration(Channel ch)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}

	uint8_t buff[4]={0, 0, 0, 0};
	buff[2]=ch + CAL_0_10V_OUT_START_ID - 1;
	buff[3]=RESET_CALIBRATION_KEY;

	if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK) {
		emit errorOccurred("Failed to reset voltage output calibration");
		return false;
	}

	QThread::msleep(100);
	return true;
}

bool MegaIndDevice::resetCurrentOutCalibration(Channel ch)
{
	if (!isValid())
	{
		return false;
	}
	QMutexLocker locker(&mutex);

	if ((ch<Channel1) || (ch>Channel4))
	{
		return false;
	}

		uint8_t buff[4]={0, 0, 0, 0};
		buff[2]=ch + CAL_4_20mA_OUT_START_ID - 1;
		buff[3]=RESET_CALIBRATION_KEY;

		if (i2cMem8Write(fd, I2C_MEM_CALIB_VALUE, buff, 4) != OK)
		{
			emit errorOccurred("Failed to reset current output calibration");
			return false;
		}

		QThread::msleep(100);
		return true;

}

MegaIndDevice::CalibrationStatus MegaIndDevice::getCalibrationStatus() const
{
	if (!isValid())
	{
		return CalibrationUnknown;
	}
	QMutexLocker locker(&mutex);

	uint8_t buff[1];
	if (i2cMem8Read(fd, I2C_MEM_CALIB_STATUS, buff, 1) != OK)
	{
		return CalibrationUnknown;
	}

	switch (buff[0])
	{
		case 0:
			return CalibrationInProgress;
		case 1:
			return CalibrationDone;
		case 2:
			return CalibrationError;
		default:
			return CalibrationUnknown;
	}
}

void MegaIndDevice::startPolling()
{
	if (pollConfig.analogInputsPollingIntervalms>0)
	{
		if (isValid())
		{
			qDebug() << "Initializing analog input modes...";
			for (int ch=1; ch <= 4; ch++)
			{
				// 0-10V mode for all channels
				setUinType(fd, ch, 0);
			}
			qDebug() << "Analog input modes initialized (0-10V)";
		}
		qDebug() << "Analog inputs polling started";
		analogInputsPollingTimer->start(pollConfig.analogInputsPollingIntervalms);
	}
	else
	{
		qDebug() << "Analog inputs polling disabled";
		analogInputsPollingTimer->stop();
	}

	if (pollConfig.optoInputsPollingIntervalms>0)
	{
		qDebug() << "Optical inputs polling started";
		optoInputsPollingTimer->start(pollConfig.optoInputsPollingIntervalms);
	}
	else
	{
		qDebug() << "Optical inputs polling disabled";
		optoInputsPollingTimer->stop();
	}

	if (pollConfig.oneWirePollingIntervalms>0)
	{
		qDebug() << "OneWire devices polling started";
		oneWireDevicesPollingTimer->start(pollConfig.oneWirePollingIntervalms);
	}
	else
	{
		qDebug() << "OneWire devices polling disabled";
		oneWireDevicesPollingTimer->stop();
	}

	if (pollConfig.countersPollingIntervalms>0)
	{
		qDebug() << "Counters polling started";
		countersPollingTimer->start(pollConfig.countersPollingIntervalms);
	}
	else
	{
		qDebug() << "Counters devices polling disabled";
		countersPollingTimer->stop();
	}

	emit pollingStarted();
}

void MegaIndDevice::stopPolling()
{
	qDebug() << "Analog inputs polling stopped";
	analogInputsPollingTimer->stop();
	qDebug() << "Optical inputs polling stopped";
	optoInputsPollingTimer->stop();
	qDebug() << "OneWire devices polling stopped";
	oneWireDevicesPollingTimer->stop();
	qDebug() << "Counters devices polling stopped";
	countersPollingTimer->stop();
	emit pollingStopped();
}

PollingConfig MegaIndDevice::pollingConfig() const
{
	return pollConfig;
}

void MegaIndDevice::setPollingConfig(const PollingConfig &config)
{
	pollConfig=config;
}

int MegaIndDevice::setUinType(int dev, uint8_t ch, uint8_t val)
{
	uint8_t buf=0;
	uint8_t needWrite=0;

	if (OK != i2cMem8Read(dev, I2C_MEM_PM_IN_SW, &buf, 1))
	{
		printf("Fail to check input type\n");
		return ERROR;
	}
	if (val == 0)
	{
		if ((buf & (1 << (ch - 1))) != 0)
		{
			buf &= ~ (1 << (ch - 1));
			needWrite=1;
		}
	}
	else
	{
		if ((buf & (1 << (ch - 1))) == 0)
		{
			buf |= 1 << (ch - 1);
			needWrite=1;
		}
	}
	if (needWrite)
	{
		if (OK != i2cMem8Write(dev, I2C_MEM_PM_IN_SW, &buf, 1))
		{
			printf("Fail to set input type!\n");
			return ERROR;
		}
	}
	return OK;
}

int MegaIndDevice::val16Get(const int dev, int baseAdd, int ch, float *val) const
{
	uint8_t buff[2]={0, 0}, add=0;
	s16 raw=0;
	if (ch<CHANNEL_NR_MIN)
	{
		return ERROR;
	}
	if ( (baseAdd + 2 * (ch - 1)) >= I2C_MEM_SIZE)
	{
		return ERROR;
	}
	add=baseAdd + 2 * (ch - 1);
	QMutexLocker locker(&mutex);
	if (OK != i2cMem8Read(dev, add, buff, 2))
	{
		return ERROR;
	}
	memcpy(&raw, buff, 2);
	*val=(float)raw;
	return OK;
}

int MegaIndDevice::val16Set(int dev, int baseAdd, int ch, float scale, float val) const
{
	uint8_t buff[2]={0, 0};
	s16 raw=0;
	uint8_t add=0;
	if (ch<CHANNEL_NR_MIN)
	{
		return ERROR;
	}
	if ( (baseAdd + 2 * (ch - 1)) >= I2C_MEM_SIZE)
	{
		return ERROR;
	}
	add=baseAdd + 2 * (ch - 1);
	raw=(s16)ceil(val * scale); //transform to milivolts
	memcpy(buff, &raw, 2);
	if (OK != i2cMem8Write(dev, add, buff, 2))
	{
		printf("Fail to write!\n");
		return ERROR;
	}
	return OK;
}

void MegaIndDevice::processOptoInputs()
{
	uint8_t currentStates=getOptoInputs();
	int ch;
	if (!isValid())
	{
		return;
	}

	if (!cached.optoStatesInit)
	{
		cached.optoStates=currentStates;
		cached.optoStatesInit=true;
		for (ch=0; ch<4; ch++)
		{
			cached.optoFrequencies[ch]=0;
			bool oState=(cached.optoStates >> ch) & 0x01;
			emit optoInputChanged(static_cast<Channel>(ch + 1), oState);
		}
		return;
	}

	if (currentStates != cached.optoStates)
	{
		for (ch=0; ch<4; ch++)
		{
			bool oldState=(cached.optoStates >> ch) & 0x01;
			bool newState=(currentStates >> ch) & 0x01;
			if (oldState != newState)
			{
				emit optoInputChanged(static_cast<Channel>(ch + 1), newState);
			}
		}
		cached.optoStates=currentStates;
	}

	for (ch=1; ch <= 4; ch++)
	{
		float freq=getOptoFrequency(static_cast<Channel>(ch));
		float &cachedFreq=cached.optoFrequencies[ch-1];
		if (std::abs(freq - cachedFreq)>1.0f)
		{
			emit optoFrequencyChanged(static_cast<Channel>(ch), freq);
			cachedFreq=freq;
		}
	}
}

void MegaIndDevice::processCounters()
{
	if (!isValid())
	{
		return;
	}
	for (int ch=1; ch <= 4; ch++)
	{
		uint32_t currentCount=getOptoCounter(static_cast<Channel>(ch));
		emit counterChanged(static_cast<Channel>(ch), currentCount);
	}
}

void MegaIndDevice::processAnalogInputs()
{
	if (!isValid())
	{
		return;
	}
	for (int ch=1; ch <= 4; ch++)
	{
		float voltageReading=getVoltageIn(static_cast<Channel>(ch));
		if (voltageReading <= -1.25 || voltageReading>10.25)
		{
			continue;
		}
		emit voltageInChanged(static_cast<Channel>(ch), voltageReading);
	}
	for (int ch=1; ch <= 4; ch++)
	{
		float voltagePMReading=getVoltageInPM(static_cast<Channel>(ch));
		if (voltagePMReading <= -10.25 || voltagePMReading>10.25)
		{
			continue;
		}
		emit voltageInPMChanged(static_cast<Channel>(ch), voltagePMReading);
	}
	for (int ch=1; ch <= 4; ch++)
	{
		float currentReading=getCurrentIn(static_cast<Channel>(ch));
		if (currentReading <= -1.25 || currentReading>20.0)
		{
			continue;
		}
		emit currentInChanged(static_cast<Channel>(ch), currentReading);
	}
}

void MegaIndDevice::processOneWireDevices()
{
	if (!isValid())
	{
		return;
	}
	if (!cached.oneWireInit)
	{
		int SensorCount=oneWireSensorCount();

		for (int i=0; i<SensorCount && i<16; i++)
		{
			cached.oneWireIds[i]=getOneWireRomId(i);
		}
		cached.oneWireInit=true;
		for (int i=SensorCount; i<cached.oneWireCount && i<16; i++)
		{
			if (cached.oneWireIds[i] != 0)
			{
				emit oneWireSensorLost(i, cached.oneWireIds[i]);
				cached.oneWireIds[i]=0;
			}
		}
		for (int i=cached.oneWireCount; i<SensorCount && i<16; i++)
		{
			uint64_t newId=getOneWireRomId(i);
			if (newId != 0)
			{
				emit oneWireSensorFound(i, newId);
				cached.oneWireIds[i]=newId;
			}
		}
		cached.oneWireCount=SensorCount;
		return;
	}
	float currentTemp;
	for (int i=0; i<cached.oneWireCount && i<16; i++)
	{
		currentTemp=getOneWireTemp(i);
		if (currentTemp>-999.0f)
		{
			emit temperatureChanged(0, currentTemp);
		}
	}
}

}
