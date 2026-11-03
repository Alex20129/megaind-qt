#ifndef MEGAIND_DEVICE_HPP
#define MEGAIND_DEVICE_HPP

#include <QObject>
#include <QMutex>
#include <QDateTime>
#include <QTimer>

#define DEFAULT_OPTO_INPUTS_POLLING_INTERVAL_MS		450
#define DEFAULT_ANALOG_INPUTS_POLLING_INTERVAL_MS	400
#define DEFAULT_ONEWIRE_POLLING_INTERVAL_MS			450
#define DEFAULT_COUNTERS_POLLING_INTERVAL_MS		800

namespace MegaInd
{

struct PollingConfig
{
	int optoInputsPollingIntervalms=DEFAULT_OPTO_INPUTS_POLLING_INTERVAL_MS;
	int analogInputsPollingIntervalms=DEFAULT_ANALOG_INPUTS_POLLING_INTERVAL_MS;
	int oneWirePollingIntervalms=DEFAULT_ONEWIRE_POLLING_INTERVAL_MS;
	int countersPollingIntervalms=DEFAULT_COUNTERS_POLLING_INTERVAL_MS;
};

class MegaIndDevice : public QObject
{
	Q_OBJECT

	QTimer *optoInputsPollingTimer = nullptr;
	QTimer *analogInputsPollingTimer = nullptr;
	QTimer *oneWireDevicesPollingTimer = nullptr;
	QTimer *countersPollingTimer = nullptr;

	PollingConfig pollConfig;

	int setUinType(int dev, uint8_t ch, uint8_t val);
	int val16Get(const int dev, int baseAdd, int ch, float *val) const;
	int val16Set(int dev, int baseAdd, int ch, float scale, float val) const;

	int fd = -1;

	bool isValid() const
	{
		return fd > 0;
	}

	mutable QMutex mutex;

	// Voltage/Current conversions
	static constexpr float VOLT_SCALE = 1000.0f;  // mV to V
	static constexpr float CURRENT_SCALE = 1000.0f; // mA scaling

	// Calibration constants (from megaind.h) - using macros directly
	// CALIBRATION_KEY = 0xaa (defined in megaind.h)
	// RESET_CALIBRATION_KEY = 0x55 (defined in megaind.h)
	static constexpr uint8_t I2C_MEM_CALIB_VALUE = 60;
	static constexpr uint8_t I2C_MEM_CALIB_STATUS = 64;

	// Calibration channel IDs (from megaind.h)
	static constexpr uint8_t CAL_0_10V_OUT_START_ID = 1;
	static constexpr uint8_t CAL_4_20mA_OUT_START_ID = 5;
	static constexpr uint8_t CAL_0_10V_IN_START_ID = 9;
	static constexpr uint8_t CAL_4_20mA_IN_START_ID = 17;

	static constexpr uint8_t I2C_MEM_OPTO_RISING_ENABLE = 0x5F;
	static constexpr uint8_t I2C_MEM_OPTO_FALLING_ENABLE = 0x60;
	static constexpr uint8_t I2C_MEM_OPTO_FREQ1 = 0x9E;
	static constexpr uint8_t I2C_MEM_OD1_FREQ = 0xAE;

	// RS485/Modbus (from megaind.h)
	static constexpr uint8_t I2C_MODBUS_SETINGS_ADD = 0x41;

	struct CachedValues
	{
		uint8_t optoStates = 0;
		bool optoStatesInit = false;
		float optoFrequencies[4] = {0, 0, 0, 0};
		//float temperatures[16];
		uint64_t oneWireIds[16];
		int oneWireCount = 0;
		bool oneWireInit = false;
		CachedValues()
		{
			for (int i = 0; i < 16; i++)
			{
				oneWireIds[i] = 0;
			}
		}
	} cached;

public:
	enum BoardId
	{
		Board0 = 0,
		Board1,
		Board2,
		Board3,
		Board4,
		Board5,
		Board6,
		Board7
	};
	Q_ENUM(BoardId)

	enum Channel
	{
		Channel1 = 1,
		Channel2,
		Channel3,
		Channel4
	};
	Q_ENUM(Channel)

	struct BoardInfo
	{
		int firmwareMajor=0;
		int firmwareMinor=0;
		int temperature=0;      // C
		float powerVoltage=0.0F;   // V
		float raspberryVoltage=0.0F; // V
		float batteryVoltage=0.0F; // V
	};

	BoardId mBoardId = MegaIndDevice::Board0;

	static constexpr uint32_t OpenDrainDutyCycleMax=10000;

	explicit MegaIndDevice(BoardId board_id = Board0, QObject *parent = nullptr);
	~MegaIndDevice();

	bool open();
	void close();
	bool isConnected() const;
	BoardId boardId() const;

	BoardInfo getBoardInfo() const;

	bool setVoltageOut(Channel ch, float voltage); // 0-10V
	float getVoltageOut(Channel ch) const;
	bool setCurrentOut(Channel ch, float current); // 4-20mA
	float getCurrentOut(Channel ch) const;

	// Analog inputs
	float getVoltageIn(Channel ch) const;     // 0-10V
	float getVoltageInPM(Channel ch) const;   // ±10V
	float getCurrentIn(Channel ch) const;     // 4-20mA
	float getFrequencyIn(Channel ch) const;   // Hz

	// Digital inputs (opto-isolated)
	enum EdgeDetection
	{
		EdgeNone = 0,
		EdgeRising = 1,
		EdgeFalling = 2,
		EdgeBoth = 3
	};

	bool getOptoInput(Channel ch) const;
	uint8_t getOptoInputs() const;
	uint32_t getOptoCounter(Channel ch) const;
	bool resetOptoCounter(Channel ch);
	bool setOptoEdgeDetection(Channel ch, bool rising, bool falling);
	EdgeDetection getOptoEdgeDetection(Channel ch) const;
	float getOptoFrequency(Channel ch) const;

	// Digital outputs (open drain)
	bool setDigitalOut(Channel ch, bool state);
	bool getDigitalOut(Channel ch) const;
	bool setLed(Channel ch, bool state);
	bool getLed(Channel ch) const;

	bool setOpenDrainPwmFrequency(Channel ch, uint16_t freq); // 10-6400 Hz
	uint16_t getOpenDrainPwmFrequency(Channel ch) const;
	bool setOpenDrainPwmDuty(Channel ch, uint16_t duty);      // 0-10000 (0-100%)
	uint16_t getOpenDrainPwmDuty(Channel ch) const;

	// 1-Wire sensors
	int oneWireSensorCount();
	bool oneWireScan();
	float getOneWireTemp(int index) const;    // index: 0-15
	uint64_t getOneWireRomId(int index) const;

	// RTC
	QDateTime getRtcTime() const;
	bool setRtcTime(const QDateTime &time);

	// Watchdog
	bool reloadWatchdog();
	bool setWatchdogPeriod(uint16_t seconds);
	uint16_t getWatchdogPeriod() const;
	uint16_t getWatchdogResetCount() const;
	bool clearWatchdogResetCount();

	// RS485
	enum RS485Mode
	{
		RS485_DISABLED = 0,
		RS485_MODBUS_SLAVE = 1,
		RS485_BRIDGE = 2
	};

	RS485Mode getRS485Mode() const;
	bool setRS485Mode(RS485Mode mode);
	bool configureModbusSlave(uint32_t baudRate, uint8_t slaveAddress, uint8_t parity = 0, uint8_t stopBits = 1);

	enum CalibrationStatus
	{
		CalibrationInProgress = 0,
		CalibrationDone = 1,
		CalibrationError = 2,
		CalibrationUnknown = 3
	};

	bool calibrateVoltageIn(Channel ch, float voltage);
	bool calibrateCurrentIn(Channel ch, float current);
	bool resetVoltageInCalibration(Channel ch);
	bool resetCurrentInCalibration(Channel ch);

	bool calibrateVoltageOut(Channel ch, float voltage);
	bool calibrateCurrentOut(Channel ch, float current);
	bool resetVoltageOutCalibration(Channel ch);
	bool resetCurrentOutCalibration(Channel ch);

	CalibrationStatus getCalibrationStatus() const;

	void startPolling();
	void stopPolling();
	PollingConfig pollingConfig() const;
	void setPollingConfig(const PollingConfig &config);

private slots:
	void processOptoInputs();
	void processCounters();
	void processAnalogInputs();
	void processOneWireDevices();

signals:
	void errorOccurred(const QString &error);
	void connectionChanged(bool connected);

	void optoInputChanged(Channel channel, bool state);
	void counterChanged(Channel channel, quint32 count);
	void frequencyChanged(Channel channel, float freq);
	void optoFrequencyChanged(Channel channel, float freq);

	void voltageInChanged(Channel channel, float voltage);
	void voltageInPMChanged(Channel channel, float voltage);
	void currentInChanged(Channel channel, float current);

	void temperatureChanged(int sensorIndex, float temperature);
	void oneWireSensorLost(int sensorIndex, quint64 romId);
	void oneWireSensorFound(int sensorIndex, quint64 romId);

	void pollingStarted();
	void pollingStopped();
};

}

#endif // MEGAIND_DEVICE_HPP
