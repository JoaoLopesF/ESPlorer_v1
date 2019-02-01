/*****************************************
 * Code      : ESPlorer I - Esp32 Car robot controlled by WiFi
 * Programmer: Joao Lopes
 * Comments  : Using Web Socket Server instead Web Server to increase performance
 *             Not using JSON, do avoid extra overhead of this
 *             Camera software based on https://github.com/mudassar-tamboli/ESP32-OV7670-WebSocket-Camera
 * Versions  :
 * ------ 	---------- 		-------------------------
 * 0.1.0  	2018-10-30		First beta
 * 0.2.0	2018-11-25		Using WebServer instead AsyncWebServer (not need this more)
 * 							Using Web Socket Server to more performance
 * 0.2.1	2018-12-14		Turbo mode
 * 0.3.0	2018-12-25		Camera
 * 0.3.1	2018-12-31		Slow speed mode
 * 0.3.2	2018-01-08		Servo camera
 * 0.3.3	2018-01-22		Few adjustments
 * 0.3.4	2018-01-28		Few adjustments to avoid ESP32 crashes
 * 0.3.5    2018-01-30		Camera optional
 *****************************************/

/*
 * Source for Esplorer I
 *
 * Copyright (C) 2018  Joao Lopes https://github.com/JoaoLopesF/ESPlorer_v1
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 * TODO known issues:
 */

////// Defines

#define VERSION "0.3.4"					// Version

#define HOST_NAME "esplorer"			// Host name

#define AP_SSID "ESPlorer_AP"			// Soft AP
#define AP_PWD ""

#define HIGHER_PRIO_TASKS 7				// Higher priority for tasks
#define HIGH_PRIO_TASKS 5				// High priority for tasks
#define LOW_PRIO_TASKS 	1				// Low priority for tasks (equal loop task)

//#define MAIN_TASK_DELAY 5				// Delay for main task

// Times

#define TIME_TO_INFO 3					// Time to process info (eg. camera fps, ...)
#define TIME_TO_RESTART_CAM 5			// Time to restart camera if not sendind data in this time

// Pinouts

#define PIN_LED_STATUS 5				// For led of status

// Motors

#define MOTOR_GPIO_IN1 18
#define MOTOR_GPIO_IN2 19
#define MOTOR_GPIO_IN3 25
#define MOTOR_GPIO_IN4 26

#define MOTOR_SPEED_MIN 25				// Minimum speed to move motor

#define DIF_SPEED_TURN 40				// Difference of speed to turn left/right

#define MOTOR_LEFT_MAX  80				// Maximum for left motor
#define MOTOR_RIGHT_MAX  80				// Maximum for right motor

#define MOTOR_LEFT_MAX_TURBO  100		// Maximum for left motor - turbo
#define MOTOR_RIGHT_MAX_TURBO 100		// Maximum for right motor - turbo

// Camera ?

#define CAMERA true

#ifdef CAMERA

// Servo for camera

#define PIN_SERVO_CAM 23				// Pin for camera servo
#define CH_SERVO_CAM 1					// Channel for camera servo
#define CAM_SERVO_MIN 15				// Minumum degrees for servo camera
#define CAM_SERVO_MAX 75				// Maximum degrees for servo camera

// Camera OV 7670

#define CAM_SIOD 21 					// SDA
#define CAM_SIOC 22 					// SCL

#define CAM_VSYNC 34
#define CAM_HREF 35

#define CAM_XCLK 32
#define CAM_PCLK 33

#define CAM_D0 27
#define CAM_D1 17
#define CAM_D2 16
#define CAM_D3 15
#define CAM_D4 14
#define CAM_D5 13
#define CAM_D6 12
#define CAM_D7 4

#endif // CAMERA

// Lights

#define PIN_LIGHTS 2				// Pin for car lights (leds)

// Debug

#define USE_REMOTEDEBUG true

#ifdef USE_REMOTEDEBUG
#define debugV(fmt, ...) rdebugVln(fmt, ##__VA_ARGS__)
#define debugD(fmt, ...) rdebugDln(fmt, ##__VA_ARGS__)
#define debugI(fmt, ...) rdebugIln(fmt, ##__VA_ARGS__)
#define debugE(fmt, ...) rdebugEln(fmt, ##__VA_ARGS__)
#define debugA(fmt, ...) rdebugVln(fmt, ##__VA_ARGS__)
#endif

// Maximum clients for socket server

//#define WEBSOCKETS_SERVER_CLIENT_MAX  (1)

/////// Includes

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "esp_system.h"
#include "esp_wifi.h"

//#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/cpu.h"
#include "soc/rtc_cntl_reg.h"
#include "rom/ets_sys.h"
//#include "esp_system_internal.h"
#include "driver/rtc_cntl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <WiFi.h>
#include <WebServer.h>

#include "SPIFFS.h"

#include <ArduinoOTA.h>

#include <WebSockets.h>			// https://github.com/Links2004/arduinoWebSockets
#include <WebSocketsClient.h>
#include <WebSocketsServer.h>

#ifdef USE_REMOTEDEBUG
#include "RemoteDebug.h"  		// https://github.com/JoaoLopesF/RemoteDebug
#else
#include "SerialDebug.h"  		// https://github.com/JoaoLopesF/SerialDebug
#endif

#include "ESP32MotorControl.h" 	// https://github.com/JoaoLopesF/ESP32MotorControl

#include "ESP32Servo.h"			// ESP32-Arduino-Servo library

// This project

#include "Util.h" 	// Utilities of this project
#include "Fields.h"

#ifdef CAMERA

#include "OV7670.h"

#endif

// C

extern "C" {
	int rom_phy_get_vdd33();
}

/////// Variables

// Main task delay (can be changed by telnet - remote debug)

uint8_t mMainTaskDelay = 10;				// Delay for main task

// Loop taksdelay (can be changed by telnet - remote debug)

uint8_t mLoopTaskDelay = 50;				// Delay for loop task

// Socket server

WebSocketsServer mWebSocketServer(81);    	// Websocket server on port 81

boolean mConnected = false;					// Connected ?

uint16_t mCountRecvWSData = 0;				// Counter of receipt data
uint16_t mLastCountRecvWSData = 1;			// Last ounter of receipt data

// Web Server

WebServer mWebServer(80);

#ifdef CAMERA
// Camera OV 7670

OV7670 *mCamera;
#endif

#ifdef USE_REMOTEDEBUG
// RemoteDebug

RemoteDebug Debug;
#endif

// MotorControl instance

ESP32MotorControl mMotorControl = ESP32MotorControl();

// Motors

uint8_t mMotorSpeed[2] = {0,0};			// Motors actual speed
boolean mMotorForward[2] = {true, true};// Motors forward ?

uint8_t mMotorSpeedMode = 1; 			// Speed mode: 1-1/1(normal) 2->1/2, 3->1/3, 4-1/4, ...
boolean mMotorResetSlowMode = false;	// Reset slow mode ?
uint16_t mMotorTimeSlowMode = 10;		// Time in millis for each step in slow mode

// Camera servo control
#ifdef CAMERA

ESP32Servo servo;						// Servo for camera

uint8_t mCameraDegrees = 45; 			// Camera servo degreess - starts with medium position

// Camera

uint8_t  mCameraFPS = 0;				// Camera FPS
uint8_t  mCountCameraFrames = 0;		// Counter of camera frames
boolean  mCameraDebug = false; 			// Camera debug ?
uint32_t mLastCameraSend = 0;			// Last time of send data for camera

#endif

// Messages

boolean mSendInfo = false;				// Send info ?

// Led of status

uint32_t mTimeLedStatus = 0;			// Time to turn on/off the led of status

#ifdef PIN_LIGHTS
// Lights (leds)

boolean mLightsOn = false;				// Lights of robot on ?
#endif

// Tasks

static TaskHandle_t xTaskMainHandler = NULL;	// Main task
#ifdef CAMERA
static TaskHandle_t xTaskCameraHandler = NULL;	// For Camera
#endif

// Mutex

#ifdef CAMERA
portMUX_TYPE mCamMutex = portMUX_INITIALIZER_UNLOCKED; // Mutex for camera task
#endif

//portMUX_TYPE mWSMutex = portMUX_INITIALIZER_UNLOCKED;  // Mutex for callback of socketserver

/////// Routines

void setup() {

	Serial.begin(230400);
	delay(200);

	Serial.println("");
	Serial.println("Setup begin...");

	// Initialize motors

	initializeMotors();

	// GPIO - Input

	// GPIO - Output

	pinMode(PIN_LED_STATUS, OUTPUT);
	digitalWrite(PIN_LED_STATUS, LOW);

#ifdef PIN_LIGHTS

	pinMode(PIN_LIGHTS, OUTPUT);
	digitalWrite(PIN_LIGHTS, LOW);
#endif

	// Effect

	for (uint8_t i=0; i<3; i++) {

		delay(250);
		digitalWrite(PIN_LED_STATUS, HIGH);

		delay(250);
		digitalWrite(PIN_LED_STATUS, LOW);
	}

	// SPIFFS

	Serial.println("Initializing SPIFFS...");

	SPIFFS.begin();

	// WiFi

	initializeWiFi();

	// Initialize web socket server

	initializeWebSocketServer();

	// Initialize web server

	initializeWebServer();

	// Initialize OTA

	initializeOTA();

#ifdef USE_REMOTEDEBUG

	// Initialize RemoteDebug

	initializeRemoteDebug();

#else

	// SerialDebug

#ifndef DEBUG_DISABLE_DEBUGGER

	// Add Functions and global variables to SerialDebug

	// Add functions that can called from SerialDebug

	//debugAddFunctionVoid(F("function"), &function); // Example for function without args
	//debugAddFunctionStr(F("function"), &function); // Example for function with one String arg
	//debugAddFunctionInt(F("function"), &function); // Example for function with one int arg

	// Add global variables that can showed/changed from SerialDebug
	// Note: Only global, if pass local for SerialDebug, can be dangerous

	debugAddGlobalUInt16_t("motorSpeed[0]", &mMotorControl.mMotorSpeed[0]);
	debugAddGlobalBoolean("motorForward[0]", &mMotorControl.mMotorForward[0]);
	debugAddGlobalUInt16_t("motorSpeed[1]", &mMotorControl.mMotorSpeed[1]);
	debugAddGlobalBoolean("motorForward[1]", &mMotorControl.mMotorForward[1]);

#endif // DEBUG_DISABLE_DEBUGGER
#endif //USE_REMOTEDEBUG

	// Create main task in CORE 1 - with higher priority

	xTaskCreatePinnedToCore(&main_Task,
				"main_Task", 10240, NULL, HIGHER_PRIO_TASKS, &xTaskMainHandler, APP_CPU_NUM);


	// Debug

	Serial.println("Setup end...");

}

// Main task

static void main_Task(void *pvParameters) {

	debugI("Init main Task");

	// Variables

	uint32_t curMillis = millis();						// Current millis time
	uint32_t timeToSec = curMillis + 1000; 				// Time to one second
	uint32_t timeToInfo = 0;							// Time to info stuff

	uint32_t timeToSlowMode = 0;  						// Time to slow mode
	uint8_t motorSlowModeStep = 1; 						// Step of slow speed: only activate motor on 1

#ifdef CAMERA

	uint8_t lastCamDegrees = 0;							// Last camera degrees
	uint32_t timeToMoveCamServo = 0;					// Time to move servo
	uint32_t timeToDettachCamServo = 0;					// Time to detach servo
#endif

	// Led of status

	digitalWrite(PIN_LED_STATUS, HIGH);
	mTimeLedStatus = curMillis + 1000;

	////// Loop

	for (;;) {

		////// Time

		// Give a time to ESP

		delay(mMainTaskDelay);

		// Current millis

		curMillis = millis();

		/////// Process in each loop

		// Process requests

		mWebSocketServer.loop();

#ifdef CAMERA
		// Camera degrees changed ?

		if (mCameraDegrees != lastCamDegrees) {

			// Move servo

			if (curMillis >= timeToMoveCamServo) { // Only if not moving now

				if (timeToDettachCamServo == 0) { // Only if not attached
					servo.attach(PIN_SERVO_CAM, CH_SERVO_CAM);
				}

				mCameraDegrees = constrain(mCameraDegrees, CAM_SERVO_MIN, CAM_SERVO_MAX);

				debugI("servo cam: %u", mCameraDegrees);

				servo.write(mCameraDegrees);

				lastCamDegrees = mCameraDegrees; // Save it
				timeToMoveCamServo = curMillis + 25;
				timeToDettachCamServo = curMillis + 50;

			}
		}

		// Camera servo dettach time ?

		if (timeToDettachCamServo > 0 && curMillis >= timeToDettachCamServo) {

		    servo.detach();

		    timeToDettachCamServo = 0;
		}
#endif // CAMERA

		///// Process for n millis

		// Motors

		if (mMotorSpeed[0] > 0 || mMotorSpeed[1] > 0) { // Only if is moving ...

			// Slow mode

			if (mMotorSpeedMode > 1) { // Slow mode ?

				// Reset it ?

				if (mMotorResetSlowMode) {

					if (mMotorTimeSlowMode < mMainTaskDelay) {
						mMotorTimeSlowMode = mMainTaskDelay;
					}
					timeToSlowMode = curMillis + mMotorTimeSlowMode;
					motorSlowModeStep = 1;
					mMotorResetSlowMode = false;
				}

				// Time to slow mode

				if (curMillis >= timeToSlowMode) { // Time to process it

					timeToSlowMode = curMillis + mMotorTimeSlowMode; // Next time

					uint8_t motorSpeedMode = mMotorSpeedMode;

					// If is rotating, max speed mode = 2

					if (mMotorForward[0] != mMotorForward[1] && motorSpeedMode > 2) {
						motorSpeedMode = 2;
					}

					if (motorSlowModeStep == motorSpeedMode) {

						motorSlowModeStep = 1;

					} else {

						motorSlowModeStep++;
					}

					if (motorSlowModeStep == 1) {

						// Turn on it

						if (mMotorForward[0]) {
							mMotorControl.motorForward(0, mMotorSpeed[0]);
						} else {
							mMotorControl.motorReverse(0, mMotorSpeed[0]);
						}
						if (mMotorForward[1]) {
							mMotorControl.motorForward(1, mMotorSpeed[1]);
						} else {
							mMotorControl.motorReverse(1, mMotorSpeed[1]);
						}

					} else {

						// Turn off it

						mMotorControl.motorsStop();

					}
				}
			}
		}

		// Led of status

		if (curMillis >= mTimeLedStatus) {

			digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));

			mTimeLedStatus = curMillis + 1000;

		}

		///// Process for each second

		// One second ?

		if (curMillis >= timeToSec) {

			timeToSec = curMillis + 1000; // Next time

			// Connected ?

			if (mConnected) {

				// Task delete not working
//
//				// Verify camera
//
//				if (xTaskCameraHandler != NULL) {
//
//					// Not sending data a time ...
//
//					if ((curMillis - mLastCameraSend) >= (TIME_TO_RESTART_CAM * 1000)) {
//
//						// Restart camera
//
//						initializeCamera();
//					}
//				}

				// Process info (for each n secs)

				if (curMillis >= timeToInfo) {

					timeToInfo = curMillis + (TIME_TO_INFO * 1000); // Next time

					// Received data in this time ?

					uint8_t recvDataPSec = 0;

					if (mCountRecvWSData == 0 && mLastCountRecvWSData == 0) {

						debugI("not received data a time");

						// This is disabled now due crash wifi
//						// Disconnection
//
//						webSocketDisconnection("mCountRecvWSData");

//						// Reset
//
//						debugI("resetting");
//
//						digitalWrite(PIN_LED_STATUS, HIGH);
//
//						ESP.restart();

					} else {

						recvDataPSec = (uint8_t) roundf(mCountRecvWSData / (TIME_TO_INFO * 1.0f));
					}

					mLastCountRecvWSData = mCountRecvWSData; // Save it
					mCountRecvWSData = 0; // Clear it

					// ESP32 VDD

					int readVDD = rom_phy_get_vdd33();

                    float voltageCaliber = 3.317f;
                    float readPhyCaliber = 6742.0f;
                    float factorCaliber = (voltageCaliber / readPhyCaliber);

                    // Voltage readed from ADC

                    float voltageEsp32 = (roundf((readVDD * factorCaliber) * 100.0f) / 100.0f);

#ifdef CAMERA
					// Camera FPS

        			portENTER_CRITICAL(&mCamMutex);

                    mCameraFPS = (uint8_t) roundf(mCountCameraFrames / (TIME_TO_INFO * 1.0f));

					mCountCameraFrames = 0;

        			portEXIT_CRITICAL(&mCamMutex);

					mSendInfo = true;

					debugI("*** ESP32 VDD %f Camera fps (avg): %u data recv./s %u", voltageEsp32, mCameraFPS, recvDataPSec);
#else
					debugI("*** ESP32 VDD data recv./s %u", voltageEsp32, recvDataPSec);
#endif

				}
			}

		}

	}

	////// End

	// Delete task

	vTaskDelete(NULL);
	xTaskMainHandler = NULL;
}

// Task for camera

#ifdef CAMERA

static void camera_Task(void *pvParameters) {

	debugI("Init camera Task");

	////// Init camera

	debugD("camera init");

	mCamera = new OV7670(OV7670::Mode::QQVGA_RGB565, CAM_SIOD, CAM_SIOC, CAM_VSYNC,
						CAM_HREF, CAM_XCLK, CAM_PCLK, CAM_D0, CAM_D1, CAM_D2, CAM_D3, CAM_D4,
						CAM_D5, CAM_D6, CAM_D7);

	// Send status

	mWebSocketServer.sendTXT(0, "caminit");

	mLastCameraSend = millis();  // To avoid disconnection, during camera initialization

	////// Loop

	uint32_t notific;

	for (;;) {

		// Wait fo notification

		if (xTaskNotifyWait(0, 0xffffffff, &notific, portMAX_DELAY) == pdPASS) {

			// Camera send next frame

			if (mCameraDebug) debugV("get camera frame data");

			if ((mCamera->yres / I2SCamera::blockSlice) > 1) { // 30, 60, 120

				debugE("camera block > 1");
			}

			uint32_t timeCam  = millis();

			mCamera->startBlock = 1;
			mCamera->endBlock = I2SCamera::blockSlice;
			mCamera->oneFrame();

			timeCam = (millis() - timeCam);

			uint32_t timeSock = millis();

			mLastCameraSend = timeSock;

			mWebSocketServer.sendBIN(0, mCamera->frame, mCamera->xres * I2SCamera::blockSlice * 2);

			timeSock = (millis() - timeSock);

			if (mCameraDebug) debugV("frame get in %lu ms and sended in %lu ms", timeCam, timeSock);

			// Count frames

			portENTER_CRITICAL(&mCamMutex);

			mCountCameraFrames++;

			portEXIT_CRITICAL(&mCamMutex);

			// Send motors info ? (neeeds be here to avoid errors on socket in browsers)

			if (mSendInfo) {

				String info = getInfo();

				mWebSocketServer.sendTXT(0, info);

				mSendInfo = false;

			}
		}
	}

	////// End

	debugI("camera task finish");

	// Delete task

	vTaskDelete(NULL);
	xTaskCameraHandler = NULL;
}
#endif // CAMERA


void loop() {

//	// Not used - delay forever
//
//	delay(portMAX_DELAY);

	// Delay

	delay(mLoopTaskDelay);

	/////// Process with low priority

	// Web Server // TODO: optimize this

	mWebServer.handleClient();

#ifdef USE_REMOTEDEBUG

	// Remote debug over telnet

	Debug.handle();

#else

	// SerialDebug handle
	// NOTE: if in inactive mode (until receive anything from serial),
	// it show only messages of always or errors level type
	// And the overhead during inactive mode is very much low

	debugHandle();
#endif

	// ArduinoOTA handle

	ArduinoOTA.handle();

}

///// Web socket server

void initializeWebSocketServer() { // Start a WebSocket server

	// Initialize web socket server

	mWebSocketServer.begin();                 // start the websocket server
	mWebSocketServer.onEvent(webSocketEvent); // if there's an incomming websocket message, go to function 'webSocketEvent'

	Serial.println("WebSocket server started.");

}

void finalizeWebSocketServer() { // Ends a WebSocket server

	// Finalize web socket server

	mWebSocketServer.close();                 // stop the websocket server

	mConnected = false;
	mSendInfo = false;
	mLastCountRecvWSData = 1;


	debugI("WebSocket server stopped.");

}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t payloadlength) { // When a WebSocket message is received

	int blk_count = 0;
	char ipaddr[26];
	IPAddress localip;

	switch (type) {
		case WStype_DISCONNECTED:             // if the websocket is disconnected

			debugD("[%u] Disconnected!\n", num);

			// Disconnection

			if (mConnected) {
				webSocketDisconnection("event");
			}
			break;
		case WStype_CONNECTED: {              // if a new websocket connection is established
			IPAddress ip = mWebSocketServer.remoteIP(num);
			debugD("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3],
					payload);

			// Stop motors

			carStop();

			// Init variables

			mConnected = true;

			// Led de status

			digitalWrite(PIN_LED_STATUS, HIGH);
			mTimeLedStatus = millis() + 2000;

		}
			break;
		case WStype_TEXT:                     // if new text data is received
		{

			if (payloadlength == 0) {
				return;
			}

			//debugI("[%u] get Text: %s", num, payload);

			// Count receipt data

			mCountRecvWSData++;

			// led of status (turn on in short time)

			digitalWrite(PIN_LED_STATUS, HIGH);
			mTimeLedStatus = millis() + 50;

			// Process message

			String ret = webSocketProcessMessage(payload, payloadlength);

			// Send message to client

			if (ret.length() > 0) {

				mWebSocketServer.sendTXT(num, ret);
			}
		}
			break;
		case WStype_ERROR:                     // if new text data is received
			debugE("Error");
			break;
		default:
			debugE("WStype %x not handled \n", type);

	}
}

// Process message

String webSocketProcessMessage(uint8_t *payload, size_t length) {

	//// Format of messages
	//
	// Code		Format			Description
	// ----		---------------	-----------------------------------------
	// init 	:				init
	// info		:info ...		informations
	// feedback	:				feedback
	// caminit 	:				camera init
	// camframe	:				camera request next frame
	// cammmove :dir:dist		camera move: direction and distance
	// carstop	:				car stop
	// carmove	:dir:dist		car move: direction and distance
	// caropt	:option			car option
	//

	// Return message to client

	String ret = "";

	// Get a message

	String message = (char*)payload;

	// Debug

	boolean showDebug = true;

#ifdef CAMERA
	if (message.startsWith("cam") && !mCameraDebug) {
		showDebug = false;
	}
#endif

	if (showDebug) debugV("message: %s", message.c_str());

	// Get a fields

	Fields fields(message, ':');

	if (fields.size() == 0) {
		return ret;
	}

	// Message type

	String type = fields.getString(1);

	if (showDebug) debugV("type: %s", type.c_str());

	// Process types

	if (type == "init") { // Init

		// Stop motors

		carStop();

	} else if (type == "reset") { // Reset

		// Stop motors

		carStop();

		// Reset

		debugI("resetting");

		digitalWrite(PIN_LED_STATUS, HIGH);

		ESP.restart();

	} else if (type == "feedback") { // Feedback

		// Not need do anything - this is important only to mark time

	} else if (type.startsWith("cam")) { // Camera

#ifdef CAMERA

		boolean sendCameraData = false;

		if (type == "caminit") {

			// Camera init

			initializeCamera();

			sendCameraData = true;

		} else if (type == "camframe") {

			// Camera next frame

			sendCameraData = true;

		} else if (type == "cammove") {

			// Camera servo move

			if (fields.size() < 2) {
				debugE("invalid message");
				return ret;
			}

			String direction = fields.getString(2);
			uint8_t distance = fields.getInt(3);

			debugV("dir: %s speed: %u", direction.c_str(), distance);

			// Camera servo move

			camMove(direction, distance);

		} else if (type == "camend") {

			// Camera end moving

			if (mMotorControl.mMotorSpeed[0] > 0 || mMotorControl.mMotorSpeed[1] > 0) {

				// Car stop

				carStop();
			}
		}

		if (sendCameraData) {

			// Send notification to it process in separate task

			xTaskNotifyGive(xTaskCameraHandler);

		}
#else

		// Advice that no have camera

		mWebSocketServer.sendTXT(0, "camdis:");

#endif

	} else { // Car control

		if (type == "carstop") {

			debugV("carstop");

			carStop();

		} else if (type == "carmove") {

			if (fields.size() < 3) {
				debugE("invalid message");
				return ret;
			}

			String direction = fields.getString(2);
			uint8_t distance = fields.getInt(3);
			boolean turbo = (fields.getChar(4) == 'y');

			debugV("dir: %s speed: %u turbo: %c", direction.c_str(), distance, ((turbo)?'y':' '));

			// Car move

			carMove(direction, distance, turbo);

		} else if (type == "caropt") {

			// Car options

			String option = fields.getString(2);

			carOption (option);

#ifdef PIN_LIGHTS
		} else if (type == "carlights") {

			// Turn lights on/off

			mLightsOn = !mLightsOn;

			digitalWrite(PIN_LIGHTS, (mLightsOn)?HIGH:LOW);

			// Request to send info

			mSendInfo = true;
#endif
		}
	}

	return ret;
}

// Return informations to send

String getInfo() {

	// Motors

	String leftStatus = "";
	String rightStatus = "";
	uint8_t leftSpeed = 0;
	uint8_t rightSpeed = 0;

	if (mMotorSpeed[0] == 0) {
		leftStatus = "Stop";
	} else if (mMotorForward[0]) {
		leftStatus = "Forward ";
		leftSpeed = (uint8_t) roundf(mMotorSpeed[0] / mMotorSpeedMode);
		leftStatus.concat(leftSpeed);
		leftStatus.concat("%");
	} else {
		leftStatus = "Reverse ";
		leftSpeed = (uint8_t) roundf(mMotorSpeed[0] / mMotorSpeedMode);
		leftStatus.concat(leftSpeed);
		leftStatus.concat("%");
	}

	if (mMotorSpeed[1] == 0) {
		rightStatus = "Stop";
	} else if (mMotorForward[1]) {
		rightStatus = "Forward ";
		rightSpeed = (uint8_t) roundf(mMotorSpeed[1] / mMotorSpeedMode);
		rightStatus.concat(rightSpeed);
		rightStatus.concat("%");
	} else {
		rightStatus = "Reverse ";
		rightSpeed = (uint8_t) roundf(mMotorSpeed[1] / mMotorSpeedMode);
		rightStatus.concat(rightSpeed);
		rightStatus.concat("%");
	}

	// Return informations to send

	String ret = "info:";
	ret.concat(mMotorSpeedMode);
	ret.concat(':');
	ret.concat(leftSpeed);
	ret.concat(':');
	ret.concat(leftStatus);
	ret.concat(':');
	ret.concat(rightSpeed);
	ret.concat(':');
	ret.concat(rightStatus);
	ret.concat(':');
#ifdef CAMERA
	ret.concat(mCameraFPS);
	ret.concat(':');
	ret.concat(mCameraDegrees);
	ret.concat(':');
	ret.concat((mLightsOn)?1:0);
#else
	ret.concat("0:0:0");
#endif

	return ret;

}

// Web socket disconnection

void webSocketDisconnection(const char* cause) {

	debugI("cause: %s", cause);

	// Stop motors

	carStop();

	// Init variables

	mConnected = false;

	mSendInfo = false;
	mLastCountRecvWSData = 1;

	// Task delete not working

//	// Task of camera is running ?
//
//	if (xTaskCameraHandler != NULL) {
//		vTaskDelete(xTaskCameraHandler);
//		delay(1);
//		xTaskCameraHandler = NULL;
//	}

	// Disconnect socket clients

	if (mWebSocketServer.connectedClients() > 0) {

		debugI("disconnect socket client");

		mWebSocketServer.disconnect();
	}

	// Led de status

	digitalWrite(PIN_LED_STATUS, HIGH);
	mTimeLedStatus = millis() + 5000;

}

////// WiFi

void initializeWiFi() {

    ////// Starts the WiFi

	Serial.println("Initializing WiFi ...");

	// Connect router (STA mode)

    // TODO: necessary ?
	WiFi.enableSTA(true);
	delay(100);

    // Connect (SSID e password stored)

	WiFi.begin();
//	WiFi.begin("RSF-TEIA-T25", "nhandu15.");

	// Wait connection

	uint32_t timeout = millis() + 20000; // Time out

	while (WiFi.status() != WL_CONNECTED && millis() < timeout) {

		delay(250);

		Serial.print(".");
	}

	// Not connected yet ?

	if (WiFi.status() != WL_CONNECTED) {

		// SmartConfig

		WiFi.beginSmartConfig();

		// Wait for SmartConfig packet from mobile

		Serial.println("Waiting for SmartConfig.");
		while (!WiFi.smartConfigDone()) {
			delay(500);
			Serial.print(".");
		  }

		  Serial.println("");
		  Serial.println("SmartConfig received.");

		  //Wait for WiFi to connect to AP
		  Serial.println("Waiting for WiFi");
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.print(".");
		}
	}

	Serial.println("");

	// Starts soft AP

	//	WiFi.mode (WIFI_MODE_AP);
	//esp_wifi_set_ps (WIFI_PS_NONE);

	WiFi.softAP(AP_SSID, AP_PWD);

	// Set modem sleep to WiFi
	//	esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    // End

	Serial.printf("WiFi started: STA IP: %s Soft AP IP: %s\n", WiFi.localIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
}

////// Web server

// Initialize

void initializeWebServer() {

	// Serve files on SPIFFS /www

	mWebServer.onNotFound([]() {
		if(!handleFileRead(mWebServer.uri()))
			mWebServer.send(404, "text/plain", "Not found");
	});

	// Start

	mWebServer.begin();

	Serial.println("Web server started");

}

// Handle file read

boolean handleFileRead(String path) {

	// Serve files on SPIFFS /www

	path = "/www" + path;

	if (path.endsWith("/"))
		path += "index.htm";

	String contentType = getWebContentType(path);

	debugV("path: %s c.type: %s", path.c_str(), contentType.c_str());

	if (SPIFFS.exists(path)) {

		File file = SPIFFS.open(path, "r");
		size_t sent = mWebServer.streamFile(file, contentType);
		file.close();
		return true;

	} else {

		debugV("File not exist");
	}

	return false;
}

// Get Web Content type

String getWebContentType(String filename) {

	if (filename.endsWith(".htm"))
		return "text/html";
	else if (filename.endsWith(".html"))
		return "text/html";
	else if (filename.endsWith(".css"))
		return "text/css";
	else if (filename.endsWith(".js"))
		return "application/javascript";
	else if (filename.endsWith(".png"))
		return "image/png";
	else if (filename.endsWith(".gif"))
		return "image/gif";
	else if (filename.endsWith(".jpg"))
		return "image/jpeg";
	else if (filename.endsWith(".ico"))
		return "image/x-icon";
	else if (filename.endsWith(".xml"))
		return "text/xml";
	else if (filename.endsWith(".pdf"))
		return "application/x-pdf";
	else if (filename.endsWith(".zip"))
		return "application/x-zip";
	else if (filename.endsWith(".gz"))
		return "application/x-gzip";
	return "text/plain";
}

#ifdef CAMERA

////// Camera

void initializeCamera() {

	debugI("");

	// Task delete not working

//	// Task is running ?
//
//	if (xTaskCameraHandler != NULL) {
//
//		debugI("delete task");
//
//		vTaskDelete(xTaskCameraHandler);
//		delay(1);
//		xTaskCameraHandler = NULL;
//	}

	// Only if not running yet

	if (xTaskCameraHandler == NULL) {

		// Starts the task - higher priority

		debugI("create task");

		xTaskCreatePinnedToCore(&camera_Task,
					"camera_Task", 102400, NULL, HIGHER_PRIO_TASKS, &xTaskCameraHandler, APP_CPU_NUM);

	}
}
#endif

////// Car robot control

// Move by direction and distance

void carMove(String& direction, uint8_t distance, boolean turbo) {

//	// Variaveis
//
//	int8_t motorSpeed[2] = {0, 0};
//	boolean motorForward[2] = {true, true};

//	// Turbo - only for forward
//
//	if (!(direction == "up" || direction == "left-up" || direction == "")
	// Debug

	debugV("dir. = %s dist = %u turbo=%c", direction.c_str(), distance, ((turbo)?'y':' '));

	// Calculate speed from distance

	uint8_t speed[2];

	if (distance < 20) {
		speed[0] = 0;
		speed[0] = 0;
	} else {
		speed[0] = map(distance, 20, 100, MOTOR_SPEED_MIN, ((turbo)?MOTOR_LEFT_MAX_TURBO:MOTOR_LEFT_MAX));
		speed[1] = map(distance, 20, 100, MOTOR_SPEED_MIN, ((turbo)?MOTOR_RIGHT_MAX_TURBO:MOTOR_RIGHT_MAX));
	}

	// Speed in line (max)

	uint16_t speedInLine = 0;
	if (mMotorControl.mMotorForward[0] == mMotorControl.mMotorForward[1]) {
		speedInLine = ((mMotorControl.mMotorSpeed[0] >= mMotorControl.mMotorSpeed[1]) ? mMotorControl.mMotorSpeed[0] : mMotorControl.mMotorSpeed[1]);
	}

	debugV("speed: %u %u actual: %u", speed[0], speed[1], speedInLine);

	// Stopped ?

	boolean stopped = (mMotorControl.mMotorSpeed[0] == 0 && mMotorControl.mMotorSpeed[1] == 0);

	// Act in motors, depends on direction

	if (direction == "up") {

		mMotorSpeed[0] = speed[0];
		mMotorSpeed[1] = speed[1];
		mMotorForward[0] = true;
		mMotorForward[1] = true;

	} else if (direction == "down") {

		mMotorSpeed[0] = speed[0];
		mMotorSpeed[1] = speed[1];
		mMotorForward[0] = false;
		mMotorForward[1] = false;

	} else if (direction == "left") {

		if (stopped || speedInLine == 0) { // Rotate

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = speed[1];
			mMotorForward[0] = false;
			mMotorForward[1] = true;

		} else { // Turn left

			mMotorSpeed[1] = speed[1];
//			mMotorSpeed[0] = speed - DIF_SPEED2;
//			if (mMotorSpeed[0] < 0) {
//				mMotorSpeed[0] = 0;
//			}
			mMotorSpeed[0] = MOTOR_SPEED_MIN;
			mMotorForward[0] = mMotorControl.mMotorForward[0];
			mMotorForward[1] = mMotorControl.mMotorForward[1];
		}

	} else if (direction == "right") {

		if (stopped || speedInLine == 0) { // Rotate

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = speed[1];
			mMotorForward[0] = true;
			mMotorForward[1] = false;

		} else { // Turn right

			mMotorSpeed[0] = speed[0];
//			mMotorSpeed[1] = speed - DIF_SPEED2;
//			if (mMotorSpeed[1] < 0) {
//				mMotorSpeed[1] = 0;
//			}
			mMotorSpeed[1] = MOTOR_SPEED_MIN;
			mMotorForward[0] = mMotorControl.mMotorForward[0];
			mMotorForward[1] = mMotorControl.mMotorForward[1];
		}

	} else if (direction == "left-up") {

		if (stopped || speedInLine == 0) { // Rotate

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = speed[1];
			mMotorForward[0] = false;
			mMotorForward[1] = true;

		} else { // Turn left

			mMotorSpeed[1] = speed[1];
			mMotorSpeed[0] = (speed[0] > DIF_SPEED_TURN) ? (speed[0] - DIF_SPEED_TURN): (DIF_SPEED_TURN / 2);
			if (mMotorSpeed[0] < 0) {
				mMotorSpeed[0] = 0;
			}
			mMotorForward[0] = true;
			mMotorForward[1] = true;
		}

	} else if (direction == "left-down") {

		if (stopped || speedInLine == 0) { // Rotate

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = speed[1];
			mMotorForward[0] = false;
			mMotorForward[1] = true;

		} else { // Turn left

			mMotorSpeed[1] = speed[1];
			mMotorSpeed[0] = (speed[0] > DIF_SPEED_TURN) ? (speed[0] - DIF_SPEED_TURN): (DIF_SPEED_TURN / 2);
			if (mMotorSpeed[0] < 0) {
				mMotorSpeed[0] = 0;
			}
			mMotorForward[0] = false;
			mMotorForward[1] = false;
		}

	} else if (direction == "right-up") {

		if (stopped || speedInLine == 0) { // Rotate

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = speed[1];
			mMotorForward[0] = true;
			mMotorForward[1] = false;

		} else { // Turn right

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = (speed[1] > DIF_SPEED_TURN) ? (speed[1] - DIF_SPEED_TURN): (DIF_SPEED_TURN / 2);
			if (mMotorSpeed[1] < 0) {
				mMotorSpeed[1] = 0;
			}
			mMotorForward[0] = true;
			mMotorForward[1] = true;
		}

	} else if (direction == "right-down") {

		if (stopped || speedInLine == 0) { // Rotate

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = speed[1];
			mMotorForward[0] = true;
			mMotorForward[1] = false;

		} else { // Turn right

			mMotorSpeed[0] = speed[0];
			mMotorSpeed[1] = (speed[1] > DIF_SPEED_TURN) ? (speed[1] - DIF_SPEED_TURN): (DIF_SPEED_TURN / 2);
			if (mMotorSpeed[1] < 0) {
				mMotorSpeed[1] = 0;
			}
			mMotorForward[0] = false;
			mMotorForward[1] = false;
		}
	}

	// Move the car

	if (mMotorSpeed[0] > 0 || mMotorSpeed[1] > 0) {

		debugV("motorA: speed %u forward %d | motor B: speed %u forward %d",
				mMotorSpeed[0], mMotorForward[0], mMotorSpeed[1], mMotorForward[1]);

		mMotorSpeed[0] = constrain(mMotorSpeed[0], 0, 100);
		mMotorSpeed[1] = constrain(mMotorSpeed[1], 0, 100);


		if (mMotorForward[0]) {
			mMotorControl.motorForward(0, mMotorSpeed[0]);
		} else {
			mMotorControl.motorReverse(0, mMotorSpeed[0]);
		}
		if (mMotorForward[1]) {
			mMotorControl.motorForward(1, mMotorSpeed[1]);
		} else {
			mMotorControl.motorReverse(1,mMotorSpeed[1]);
		}

		// Request to send info

		mSendInfo = true;

	} else {

		// Stop car

		carStop();

	}
}

// Stop car

void carStop() {

	// Stop motors

	mMotorControl.motorsStop();

	// Set variables

	mMotorSpeed[0] = 0;
	mMotorForward[0] = true;
	mMotorSpeed[1] = 0;
	mMotorForward[1] = true;

	// Request to send info

	mSendInfo = true;

}


// Car option

void carOption (String& option) {

	debugD("option = %s", option.c_str());

	if (option == "slow") {

		// Slower speed mode

		mMotorSpeedMode++;
		mMotorResetSlowMode = true;

		debugD("slow -> mode=%u", mMotorSpeedMode);

		// Request to send info

		mSendInfo = true;

	} else if (option == "fast") {

		// Faster speed mode

		if (mMotorSpeedMode > 1) {

			mMotorSpeedMode--;
			mMotorResetSlowMode = true;

			debugD("faster -> mode=%u", mMotorSpeedMode);

			// Request to send info

			mSendInfo = true;
		}
	}
}

////// Camera robot control

#ifdef CAMERA

// Move by direction and distance

void camMove(String& direction, uint8_t distance) {

	debugV("dir. = %s dist = %u", direction.c_str(), distance);

	// Act in motors, depends on direction

	if (direction == "up") { // Camera up

		mCameraDegrees = map(distance, 0, 100, 45, CAM_SERVO_MAX);

		mSendInfo = true; // Request to send info

	} else if (direction == "down") { // Camera down

		mCameraDegrees = map((100 - distance), 0, 100, CAM_SERVO_MIN, 45);

		mSendInfo = true; // Request to send info

	} else  { // Rotate car -> rotate camera

		// Stop before

		if (mMotorControl.mMotorSpeed[0] > 0 || mMotorControl.mMotorSpeed[1] > 0) {

			carStop();
		}

		// Rotate

		carMove(direction, distance, false);
	}
}

#endif // CAMERA

////// OTA

void initializeOTA() {

	// ArduinoOTA

	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";
			Serial.println("Start updating " + type);
		}).onEnd([]() {
		Serial.println("\nEnd");
	}).onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	}).onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

	ArduinoOTA.begin();

}

// Initialize Motors

void initializeMotors() {

	// Attach motors of robot

	Serial.println("Initializing motors ..");

	mMotorControl.attachMotors(MOTOR_GPIO_IN1, MOTOR_GPIO_IN2, MOTOR_GPIO_IN3, MOTOR_GPIO_IN4);

	mMotorControl.motorsStop();

//mMotorControl.motorFullForward(0);
//mMotorControl.motorFullForward(1);
	Serial.println("Motors initialized");
}

#ifdef USE_REMOTEDEBUG

/////// RemoteDebug

void initializeRemoteDebug() {

	// RemoteDebug

	Debug.begin(HOST_NAME);// Initiaze the telnet server

//	Debug.setPassword("r3m0t0."); // Set password to connect on telnet

	Debug.setResetCmdEnabled(true);// Enable the reset command

	//Debug.showDebugLevel(false); // To not show debug levels
	//Debug.showTime(true); // To show time
	//Debug.showProfiler(true); // To show profiler - time between messages of Debug
	// Good to "begin ...." and "end ...." messages

	Debug.showProfiler(true);// Profiler
	Debug.showColors(true);// Colors

	Debug.setSerialEnabled(true); // if you wants serial echo - only recommended if ESP8266 is plugged in USB

	String helpCmd = "ota - wait upload via OTA\n";
	helpCmd.concat("mt [n] - main task delay to n\n");
	helpCmd.concat("lt [n] - loop task delay to n\n");
	helpCmd.concat("st [n] - motor slow time to n\n");
	helpCmd.concat("cd [on/off] - camera debug\n");
//	helpCmd.concat("mt stop - stop all motors");

	Debug.setHelpProjectsCmds(helpCmd);
	Debug.setCallBackProjectCmds(&processCmdRemoteDebug);

}

// Process commands from RemoteDebug

void processCmdRemoteDebug() {

	// mt speed [f/r] [a/b] - motor speed
	// mt stop - stop all motors

	String lastCmd = Debug.getLastCommand();

	Fields fields(lastCmd, ' ');

	if (fields.size() == 0) {
		return;
	}
	String command = fields.getString(1);

	rdebugVln("lastCmd: %s cmd: %s", lastCmd.c_str(), command.c_str());

	if (command == "mt") { // Main task delay time

		if (fields.size() >= 2) {

			if (fields.isNum(2)) {

				mMainTaskDelay = fields.getInt(2);
			}
		}

		rdebugDln("main task delay time -> %u", mMainTaskDelay);

	} else if (command == "lt") { // Loop task delay time

			if (fields.size() >= 2) {

				if (fields.isNum(2)) {

					mLoopTaskDelay = fields.getInt(2);
				}
			}

			rdebugDln("loop task delay time -> %u", mLoopTaskDelay);

	} else if (command == "st") { // Motor slow time

		if (fields.size() >= 2) {

			if (fields.isNum(2)) {

				mMotorTimeSlowMode = fields.getInt(2);
				mMotorResetSlowMode = true;

			}
		}

		rdebugDln("motor slow time -> %u",mMotorTimeSlowMode);

#ifdef CAMERA

	} else if (command == "cd") { // Camera debug

		if (fields.size() >= 2) {

			mCameraDebug = (fields.getString(2) == "on");

		}

		rdebugDln("camera debug %s", ((mCameraDebug) ? "on" : "off"));
#endif

	} else if (command == "ota") {

		// Wait upload via OTA

		waitUploadOTA();

	}
}

// Wait OTA upload (better performance on this)

void waitUploadOTA() {

	debugA("*** Waiting update via OTA ...");

	for(;;) {

		// Update over air (OTA)

		ArduinoOTA.handle();

		// Give a time for o ESP

		yield();

	}
}

#endif

////////// Routines discontinued


//	if (command == "mt") { // motor
//
//		String firstOption = getFieldStr(lastCmd, 2, " ");
//
//		if (firstOption.length() == 0) {
//
//#ifdef MOTOR_ENCODERS
//			rdebugD("Motor A: speed: %u dir.: %c enc. %u | Motor B: speed: %u dir.: %c enc. %u\n",
//					mMotorSpeed[0], mMotorDirection[0], mMotorEncoderSec[0],
//					mMotorSpeed[1], mMotorDirection[1], mMotorEncoderSec[1]);
//#else
//			rdebugD("Motor A: speed: %u dir.: %c | Motor B: speed: %u dir.: %c \n",
//					mMotorSpeed[0], mMotorDirection[0],
//					mMotorSpeed[1], mMotorDirection[1]);
//#endif
//			return;
//
//		} else if (firstOption == "stop") {
//
//			rdebugD("Stopping motors\n");
//
//			motorStop();
//
//		} else {
//
//			if (isNumeric(firstOption)) {
//
//				int16_t speed = firstOption.toInt();
//
//				if (speed == -1) {
//					rdebugEln("motor speed invalid");
//					return;
//				}
//
//				String direction = getFieldStr(lastCmd, 3, " ");
//
//				if (direction != "" && direction != "f" && direction != "r") {
//					rdebugEln("direction must be f or r");
//					return;
//				}
//
//				String motor = "";
//				uint8_t motorNumber = 2; // 2-> all
//
//				if (direction.length() > 0) {
//
//					motor = getFieldStr(lastCmd, 4, " ");
//
//					if (motor.length() > 0) {
//
//						if (motor != "a" && motor != "b") {
//							rdebugEln("motor must be a or b");
//							return;
//						}
//
//						switch (motor.charAt(0)) {
//
//							case 'a': // Motor A
//
//								motorNumber = 0;
//								mMotorDirection[0] = direction.charAt(0);
//								break;
//
//							case 'b': // Motor B
//
//								motorNumber = 1;
//								mMotorDirection[1] = direction.charAt(0);
//								break;
//						}
//					}
//				}
//
//				// Set speed
//
//				switch (motorNumber) {
//
//					case 0: // Motor A
//					case 1: // Motor B
//
//						rdebugDln("motor speed mt: %u dir: %c spd: %u", motorNumber, mMotorDirection[motorNumber], speed);
//
//						motorSpeed(motorNumber, mMotorDirection[motorNumber], speed);
//						break;
//
//					case 2: // Motor A+B
//
//						if (direction.length() == 0) {
//							direction = "f";
//						}
//
//						rdebugDln("motor speed dir: %c spd: %u", direction.charAt(0), speed);
//
//						motorSpeed(direction.charAt(0), speed);
//						break;
//				}
//
//			} else {
//
//				rdebugEln("option invalid");
//			}
//
//		}
//
//	} else if (command == "ota") {
////// End
