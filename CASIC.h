#ifndef __CASIC_H
#define __CASIC_H

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
	// ESP32 環境では FreeRTOS の vTaskDelay() を使用
	#define CASIC_DELAY(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)
#else
	// その他の環境では delay() を使用
	#define CASIC_DELAY(ms) delay(ms)
#endif

// 衛星モードおよびブートモードの列挙型
typedef enum {
	SatelliteMode_GPS = 0,
	SatelliteMode_BDS,
	SatelliteMode_Glonass,
	SatelliteMode_Galileo,
	SatelliteMode_QZSS,
} SatelliteMode;

typedef enum {
	BootMode_HotStart = 0,
	BootMode_WarmStart,
	BootMode_ColdStart,
	BootMode_FactoryStart,
	BootMode_FactoryDisableSerialRadio,
	BootMode_FactoryEnableSerialRadio,
} BootMode;

class CASIC {
public:
	CASIC(HardwareSerial &serialPort, long baudRate, uint32_t config, int rxPin, int txPin)
		: _serial(serialPort),
		_baudRate(baudRate),
		_config(config),
		_rxPin(rxPin),
		_txPin(txPin)
	{ }

	void begin() {
		_serial.begin(_baudRate, _config, _rxPin, _txPin);
	}

	// コマンド送信
	void sendCommand(const char *cmd) {
		_serial.print(cmd);
	}

	// GNSSバージョン取得
	String getGnssVersion() {
		sendCommand("$PCAS06,0*1B\r\n");
		return readResponseAfter("SW=");
	}

	// アンテナ状態取得
	String getAntennaState() {
		return readResponseAfter("ANTENNA");
	}

	// 衛星モード取得
	String getSatelliteMode() {
		sendCommand("$PCAS06,2*19\r\n");
		String mode = readResponseAfter("MO=");
		if (mode == "G")
			return "GPS";
		else if (mode == "B")
			return "BDS";
		else if (mode == "R")
			return "GLONASS";
		else if (mode == "E")
			return "GALILEO";
		else if (mode == "Q")
			return "QZSS";
		else
			return "Unknown";
	}

	void setSatelliteMode(SatelliteMode mode) {
		switch (mode) {
			case SatelliteMode_GPS:
				sendCommand("$PCAS04,1*18\r\n");
				break;
			case SatelliteMode_BDS:
				sendCommand("$PCAS04,2*1B\r\n");
				break;
			case SatelliteMode_Glonass:
				sendCommand("$PCAS04,4*1D\r\n");
				break;
			case SatelliteMode_Galileo:
				sendCommand("$PCAS04,8*11\r\n");
				break;
			case SatelliteMode_QZSS:
				sendCommand("$PCAS04,20*2B\r\n");
				break;
			default:
				break;
		}
	}

	void setSystemBootMode(BootMode mode) {
		switch (mode) {
			case BootMode_HotStart:
				sendCommand("$PCAS10,0*1C\r\n");
				break;
			case BootMode_WarmStart:
				sendCommand("$PCAS10,1*1D\r\n");
				break;
			case BootMode_ColdStart:
				sendCommand("$PCAS10,2*1E\r\n");
				break;
			case BootMode_FactoryStart:
				sendCommand("$PCAS10,3*1F\r\n");
				break;
			case BootMode_FactoryDisableSerialRadio:
				sendCommand("$PCAS10,8*14\r\n");
				break;
			case BootMode_FactoryEnableSerialRadio:
				sendCommand("$PCAS10,9*15\r\n");
				break;
			default:
				break;
		}
	}

	void standbyMode() {
		sendCommand("$PCAS12,65535*1E\r\n");
	}

	HardwareSerial& getSerial() { return _serial; }

private:
	HardwareSerial &_serial;
	long _baudRate;
	uint32_t _config;
	int _rxPin;
	int _txPin;

	// タイムアウト付きレスポンス取得
	String readResponseAfter(const char *searchStr) {
		// タイムアウト500ms
		const uint32_t timeoutMs = 500;
		uint32_t startTime = millis();
		const int bufSize = 100;
		char buf[bufSize];
		int index = 0;
		memset(buf, 0, bufSize);
		while ((millis() - startTime) < timeoutMs) {
			if (_serial.available()) {
				char d = _serial.read();
				if (d == '\n') {
					buf[index] = '\0';
					if (strstr(buf, searchStr) != NULL) {
						break;
					}
					index = 0;
					memset(buf, 0, bufSize);
				} else {
					if (index < bufSize - 1) {
						buf[index++] = d;
					}
				}
			} else {
				CASIC_DELAY(5);
			}
		}
		if ((millis() - startTime) >= timeoutMs) {
			// タイムアウトの場合は空文字列を返す
			return String("");
		}
		char *p = strstr(buf, searchStr);
		if (p != NULL) {
			p += strlen(searchStr);
		} else {
			return String("");
		}
		char *q = strstr(p, "*");
		if (q != NULL) {
			*q = '\0';
		}
		return String(p);
	}
};

#endif // __CASIC_H
