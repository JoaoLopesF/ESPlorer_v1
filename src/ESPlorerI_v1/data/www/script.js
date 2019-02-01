 /*****************************************
 * Code      : ESPlorer I - Esp32 Car robot controlled by WiFi
 * Programmer: Joao Lopes
 * Comments  : Camera software based on https://github.com/mudassar-tamboli/ESP32-OV7670-WebSocket-Camera
 * Versions  :
 * ------ 	---------- 		-------------------------
 * 0.1.0  	2018-10-30		First beta
 * 0.2.0	2018-12-26		Camera
 * 0.3.0	2018-12-30		Motor slow mode
 * 0.3.1	2019-01-22		Few adjustments
 * 0.3.2	2019-01-29		Connectting in AP too
 *****************************************/

 // URL of tests -> 192.168.0.108/~Ararekan/esplorer_robot/index.htm

////// Variables

// Timer 

var mTimerId = null;

// Motors control

var mLastDistance = 0;
var mLastDirection = "";
var mLastTurbo = false;

var mSpeedMode = 1;

// Camera control

var mLastDistanceCam = 0;
var mLastDirectionCam = "";

// Connection

var mConnected = false;
var mConnecting = false;
var mSendOnConnect = "";
var mSending = false;
var mTotSended = 0;

// Joystick

var mJoystickCamSize = 0;
var mJoystickMotorSize = 0;

// Canvas to get image data from camera (limited by memory of ESP32 to QQ-VGA)
var mXRes = 160;
var mYRes = 120;
var mCanvasCameraQQVGA;
var mCtxCameraQQVGA;

// Image data 
var mImgData;

// Canvas to display image data scaled to 4x (QQ-VGA to VGA)
var mCanvasCameraDisplay;
var mCtxCameraDisplay;

// Debug camera ?

var mDebugCamera = false;

// Web socket server

var mWS = null;

// Gauges

var mGaugeLeft;
var mGaugeRight;

// Lights on ?

var mLightsOn = false;

///////// Routines

function processOnLoad() {

	// Process on event onLoad

	log("");

	// Disable context menu - saw it in: https://stackoverflow.com/questions/3413683/disabling-the-context-menu-on-long-taps-on-android

	window.oncontextmenu = function(event) {
		event.preventDefault();
		event.stopPropagation();
		return false;
	   };
	   
	// Scroll to hide browser toolbar - saw it in: https://stackoverflow.com/questions/7517352/fullscreen-web-app-for-android

	setTimeout(function() { window.scrollTo(0, 1); }, 1);

	// Is mobile ? - based in: https://www.abeautifulsite.net/detecting-mobile-devices-with-javascript

	var isMobileCheck = {
		Android: function() {
			return navigator.userAgent.match(/Android/i);
		},
		BlackBerry: function() {
			return navigator.userAgent.match(/BlackBerry/i);
		},
		iOS: function() {
			return navigator.userAgent.match(/iPhone|iPad|iPod/i);
		},
		Opera: function() {
			return navigator.userAgent.match(/Opera Mini/i);
		},
		Windows: function() {
			return navigator.userAgent.match(/IEMobile/i);
		},
		any: function() {
			return (isMobileCheck.Android() || isMobileCheck.BlackBerry() || isMobileCheck.iOS() || isMobileCheck.Opera() || isMobileCheck.Windows());
		}
	};

	window.isMobile = isMobileCheck.any();

	log("is mobile? " + window.isMobile);
	
	// Init canvas for camera

	initCanvas();

	// Init joysticks
	
	initJoysticks();

	// Init gauges for speed of motors

	initMotorGauges(100);

	// Send initial messagem 

	log("location: " + location.hostname);
	
	sendWS("init:");

	// Debug

	log("end");

}

////// Joysticks

function requestCamControl(evt, data) {

    // Request with new camera control movement, based on virtual joystick

	log ("evt: " + evt);
    
    switch (evt) {

        case "move":

			var distance = Math.round(data.distance);

			// Calculate distance by percent

			distance = (distance.toFixed(2) / (mJoystickCamSize.toFixed(2) / 2) * 100.0).toFixed(0);

			// By angle - direction

            if (!data.angle) {
				log("no angle !!!");
				return;
			}
            var angle =  data.angle.degree;
			var direction = "";

			if (angle >= 45 && angle <= 135) direction = "up";
			else if (angle >= 225 && angle <= 315) direction = "down";
			else if (angle >= 135 && angle <= 225) direction = "left";
			else if ((angle >= 315 && angle <= 360.0) || (angle >= 0.0 && angle <= 45)) direction = "right";

			// Minimum distance

			if ((direction == "left" || direction == "right") && distance < 30) {
				return;
			}
			
			log("angle: " + angle +  " dir.: " + direction + " dist: " + distance ); 

			// Move camera - servo

			if (direction != mLastDirectionCam || distance != mLastDistanceCam) { // Only if changed

				log("send: dir.: " + direction  + " distance: " + distance);

				sendWS("cammove:" + direction + ":" + distance);

				mLastDistanceCam = distance;
				mLastDirectionCam = direction;
			}
			
            break;
    
        case "end":
            
            // Stop camera move

			log("end");

			sendWS("camend");

			mLastDistanceCam = 0;
			mLastDirectionCam = "";
            break;
    
        default:
            break;
    }

}

function requestCarControl(evt, data) {

    // Request with new car control movement, based on virtual joystick

    log ("evt: " + evt);
    
    switch (evt) {

        case "move":

			var distance = Math.round(data.distance);

			// Minimum distance

			if (distance < 20) {
				return;
			}

			// Calculate distance by percent

			distance = (distance.toFixed(2) / (mJoystickMotorSize.toFixed(2) / 2) * 100.0).toFixed(0);

			// By angle - direction

            if (!data.angle) {
				log("no angle !!!");
				return;
			}
            var angle =  data.angle.degree;
			var direction = "";

			if (angle >= 67.5 && angle <= 112.5) direction = "up";
			else if (angle >= 247.5 && angle <= 292.5) direction = "down";
			else if (angle >= 157.5 && angle <= 202.5) direction = "left";
			else if ((angle >= 337.5 && angle <= 360.0) || (angle >= 0.0 && angle <= 22.5)) direction = "right";
			else if (angle > 112.5 && angle < 157.5) direction = "left-up";
			else if (angle > 202.5 && angle < 247.5) direction = "left-down";
			else if (angle > 22.5 && angle < 67.5) direction = "right-up";
			else if (angle > 292.5 && angle < 337.5) direction = "right-down";

			var force = Math.round(data.force);
			var turbo = false;
			if (force >= 1.5) { // Go maximum power - turbo
				turbo=true;
				setText("spanJoystickMotorHeader", direction + ":" + distance + ":turbo!");
				setText("spanJoystickMotorFooter", direction + ":" + distance + ":turbo!");
			} else { // Standard power
				setText("spanJoystickMotorHeader", direction + ":" + distance);
				setText("spanJoystickMotorFooter", direction + ":" + distance);
			}

			var diffDistance = (distance - mLastDistance);
			if (diffDistance < 0) diffDistance *= -1;

			log("angle: " + angle +  " dir.: " + direction + " dist: " + distance + " dif " + diffDistance + " force: " + force); 

			// Move car

			if (direction != mLastDirection || distance != mLastDistance || turbo != mLastTurbo) { // Only if changed
//			if (direction != mLastDirection || diffDistance >=5  || turbo != mLastTurbo) { // Only if changed

				log("send: dir.: " + direction  + " distance: " + distance + " turbo:" + turbo);

				sendWS("carmove:" + direction + ":" + distance + ":" + ((turbo)?"y":""));

				mLastDistance = distance;
				mLastDirection = direction;
				mLastTurbo = turbo;
			}
			
            break;
    
        case "end":
            
            // Stop car

			log("stoping");

			setText("spanJoystickMotorHeader", "");
			setText("spanJoystickMotorFooter", "");

            sendWS("carstop:");

			mLastDistance = 0;
			mLastDirection = "";
            break;
    
        default:
            break;
    }
}

function initJoysticks() {

	// Joystick camera

	// Initialize virtual joystick

	mJoystickCamSize = 250;
	var joystick_camera = nipplejs.create({
		zone: document.getElementById('divCamera'),
		mode: 'dynamic',
		color: '#34629C',
		size: mJoystickCamSize
	});
	
	joystick_camera.on('move', function (evt, data) {
		requestCamControl("move", data);
	}).on('end', function (evt, data) {
		requestCamControl("end", data);
	});
	
	// Joystick motor

	var divJoystickMotor = document.getElementById("divJoystickMotor");

	mJoystickMotorSize = (divJoystickMotor.offsetHeight - 100);

	log("joystick motor size = " + mJoystickMotorSize);
	
    // Initialize virtual joystick

    var joystick_motor = nipplejs.create({
      zone: divJoystickMotor,
      mode: 'static',
      position: {
        left: '50%',
        bottom: '50%'
      },
      color: '#34629C',
      size: mJoystickMotorSize
	});
	
    joystick_motor.on('move', function (evt, data) {
      requestCarControl("move", data);
    }).on('end', function (evt, data) {
      requestCarControl("end", data);
    });

}


///// Gauges

function initMotorGauges(max) {

	// Init gauges for speed of motors

	log("max " + max);

	var zone = [20, 50, 80];

	if (max != 100) {

		for (i=0; i<3; i++) {
			zone[i] = Math.round(max * (zone[i] / 100.0));
			log("zone["+ i + "]: " + zone[i]);
		}
	}
	var opts = {
		angle: 0.25, /// The span of the gauge arc
		lineWidth: 0.05, // The line thickness
		pointer: {
  		  color: "#A52A2A",
		  length: 0.55, // Relative to gauge radius
		  strokeWidth: 0.035 // The thickness
		},
		colorStart: '#6FADCF',   // Colors
		colorStop: '#8FC0DA',    // just experiment with them
		strokeColor: '#E0E0E0',   // to see which ones work best for you
		// generateGradient: true,
		// highDpiSupport: true,     // High resolution support
		staticLabels: {
			font: "10px sans-serif",
			color: '#D9D9D9',
			labels: [zone[0], zone[1], zone[2]],
			fractionDigits: 0
		  },
		  staticZones: [
			 {strokeStyle: "#F0E68C", min: 0, max: zone[0]},
			 {strokeStyle: "#6B8E23", min: zone[0], max: zone[2]},
			 {strokeStyle: "#FF8C69", min: zone[2], max: 100},
		  ],
		  renderTicks: {
			divisions: 5,
			divWidth: 1.1,
			divLength: 0.7,
			divColor: "#333333",
			subDivisions: 3,
			subLength: 0.5,
			subWidth: 0.6,
			subColor: "#666666"
		  }
	  };
	  
	// var divLeft = document.getElementById('divGaugeMotorLeft'); 
	// divLeft.style.height = divLeft.style.width + "px !important";
	//divLeft.style.height = "10px";
	
	var canvasLeft = document.getElementById('canvasGaugeMotorLeft'); // your canvas element
	// canvasLeft.style.height = canvasLeft.style.width + "px";

	mGaugeLeft = new Gauge(canvasLeft).setOptions(opts); // create gauge!

	mGaugeLeft.maxValue = max; // set max gauge value
	mGaugeLeft.setMinValue(0);  // set min value
	mGaugeLeft.set(0); // set actual value
	mGaugeLeft.animationSpeed = 5;

	var canvasRight = document.getElementById('canvasGaugeMotorRight'); // your canvas element
	mGaugeRight = new Gauge(canvasRight).setOptions(opts); // create gauge!

	mGaugeRight.maxValue = max; // set max gauge value
	mGaugeRight.setMinValue(0);  // set min value
	mGaugeRight.set(0); // set actual value
	mGaugeRight.animationSpeed = 5;

	log("end");
}

///// Web sockets

function connectWS() {
	  
	// Connect to ESP32 Web Socket Server

	try {

		// if (location.hostname != "192.168.4.1") {
		// 	log("not is 192.168.4.1, not connect");
		// 	return;
		// }

		if ("WebSocket" in window) {

			// Disconnect (if connected)

			disconnectWS();

			// Connect

			mConnecting = true;

			// var addr = "ws://192.168.4.1:81";
			var addr = "ws://" + location.hostname + ":81";
		
			log ("starting addr: " + addr);
			window.mWS = new WebSocket(addr, ['arduino']);
			
			if (mWS == null) {
				setText("spanConnection", "Failed to connect");
				return;
			}

			mWS.binaryType = 'arraybuffer';

			log ("created WS");

			setText("spanConnection", "connecting ...");			
			setText("spanStatus", "");			

			// Event handler for the WebSocket connection opening
			window.mWS.onopen = function(e) {

				log("connection established");
				mConnecting = false;
				mConnected = true;

				setText("spanConnection", "connected");			
				setText("spanStatus", "");			
		
				// Send after connect ?

				if (mSendOnConnect != "") { // Send after connection ?
					log("sending: " + mSendOnConnect);  
					window.mWS.send(mSendOnConnect);
					mSendOnConnect = "";
				}

				// Start camera capture

				capture();

				// Start timer to each second

				window.mTimerId = setInterval(timerSecond, 1000);

			};
			
			// Event handler for receiving text messages
			window.mWS.onmessage = function(e) {
				//log("message received");

				processMessageWS(e.data);
			};
			
			// Event handler for errors in the WebSocket object
			window.mWS.onerror = function(e) {

				log("error: " , e);
				setText("spanStatus", "e1");	

		
			};
			
			// Event handler for closed connections
			window.mWS.onclose = function(e) {

				log("connection closed", e);
				setText("spanStatus", "c1");		

				// Disconnect

				disconnectWS();

			};

		} else {
			// The browser doesn't support WebSocket
			alert("WebSocket NOT supported by your Browser!");
		}

	} catch(err) {
		
		log("connect: err: "+ err.message);
		setText("spanStatus", "e2");			

		// Disconnect

		disconnectWS();
	}
		
}
	

function disconnectWS () {
	
	// Disconnect from WS

	log("");

	// Stop timer

	if (mTimerId) {
		clearInterval(mTimerId);
		mTimerId = null;
	}

	// Disconnect

	if (mWS != null) {
		mWS.close();
	}

	mConnecting = false;
	mConnected = false;
	mSending = false;

	setText("spanConnection", "disconnected");			

}

function timerSecond () {

	// Timer tick - each second

	// Send feedback message, if in this second, none is sended

	if (mConnected && !mSending && mTotSended == 0) {

		log("send feedback");

		sendWS("feedback");
	}

	mTotSended = 0; // Clear it
}

function processMessageWS(data) {

	// Process message received // TODO: ver text msgs

	if (data.byteLength > 100) {

		// Data from camera

		if (mDebugCamera)
			log("display: data byteLength=" + data.byteLength);

		// // Request next frame

		// sendWS("camframe");

		// Display camera data

		var bytearray = new Uint8Array(data);

		// Display camera data

		display(bytearray, data.byteLength);

		// Request next frame

		sendWS("camframe");

	} else { 

		// Another data

		var message = data + ""; // To string

		log("msg: " + message);

		if (message.startsWith("info:")) {

			log("info");

			// Informations

			var fields = message.split (':');

			// log(fields);

			if (fields.length == 9) {

				// Speed mode

				var speedMode = fields[1];

				if (speedMode != mSpeedMode) {

					mSpeedMode = speedMode; // Save it

					setText ("spanSpeedMode", "1/" + speedMode);

					// Init gauges for speed of motors

					max = Math.round(100.0 / speedMode); // set max gauge value
					log ("max = " + max);

					initMotorGauges(max);
	
				}

				// Left motor

				var speedLeft = fields[2];

				mGaugeLeft.set(speedLeft); // set actual value

				var statusLeft = fields[3];

				setText("spanMotorLeftSpeed", statusLeft);

				// Right motor

				var speedRight = fields[4];

				mGaugeRight.set(speedRight); // set actual value

				var statusRight = fields[5];

				setText("spanMotorRightSpeed", statusRight);
				
				// Camera 

				var cameraFPS = fields[6];

				var cameraDegrees = (fields[7] - 45); // 45 -> horizontal

				setHtml("spanCamera", "Camera " + cameraFPS + "fps " + cameraDegrees + "&deg;");

				// Lights

				lightsOn = (fields[8] == "1");

				if (lightsOn != mLightsOn) {

					mLightsOn = lightsOn;

					var imgLights = document.getElementById("imgLights");
					if (imgLights) {
						imgLights.src = (mLightsOn)?"lights_on.png":"lights_off.png";
					}
				}

				// Debug

				log ("speed: left " + speedLeft + " speedRight " + speedRight + " cameraFPS " + cameraFPS + " lights "  + mLightsOn);
			
			} else if (message.startsWith("caminit")) {
			
				// Camera initialization

				setText("spanCamera", "Camera initializing ...");

			} else if (message.startsWith("camdis:")) {

				// Camera disabled

				setText("spanCamera", "No camera on device");
			}
		}	
	}
}

function sendWS(message) {
	
	// log(message);

	// Send a text message to Web Socket

	if (!(message.startsWith("cam") && !mDebugCamera)) 
		log ("message=" + message);
	
	if (mConnected) { // Is connected

		//log("sending message");

		mSending = true;

		window.mWS.send(message);
		
		mSending = false;

		mTotSended++;

	} else { // No connected ?

		if (!mConnecting) {

			log("connecting before send");
			mSendOnConnect = message;

			connectWS();

		} else {

			log("ignoring, due it is connecting yet");

		}
	}
}

///// Camera

function initCanvas() {

	log("");
	
	if (!mCanvasCameraQQVGA) {
		log("init canvas QQ-VGA");
		mCanvasCameraQQVGA = document.getElementById("canvasCameraQQVGA");
		mCtxCameraQQVGA = mCanvasCameraQQVGA.getContext("2d");

		if (!mCanvasCameraQQVGA || !mCtxCameraQQVGA) {
			log("Error on get canvas QQ VGA");
		}
		// mCtxCameraQQVGA.font = "10px Comic Sans MS";
		// mCtxCameraQQVGA.fillStyle = "#FF0000";
		// mCtxCameraQQVGA.textAlign = "center";
		// mCtxCameraQQVGA.fillText("160 x 120", mCanvasCameraQQVGA.width / 2, mCanvasCameraQQVGA.height / 2);
	}

	if (!mCanvasCameraDisplay) {
		log("init canvas camera");
		mCanvasCameraDisplay = document.getElementById("canvasCameraDisplay");
		mCtxCameraDisplay = mCanvasCameraDisplay.getContext("2d");

		if (!mCanvasCameraDisplay || !mCtxCameraDisplay) {
			log("Error on get canvas display");
		}

		// mCtxCameraDisplay.font = "10px Comic Sans MS";
		// mCtxCameraDisplay.fillStyle = "#FF0000";
		// mCtxCameraDisplay.textAlign = "center";

		// Adjust div of camera

		var divCamera = document.getElementById("divCamera");
		log ( "div cam height " + divCamera.offsetHeight);
		var height = (divCamera.offsetWidth * (3.0 / 4.0));
		// divCamera.offsetHeight = height;
		divCamera.style.height = height + "px";
		log ( "div cam height " + divCamera.offsetHeight + " calc " + height);

		mCanvasCameraDisplay.width = divCamera.offsetWidth;
		mCanvasCameraDisplay.height = divCamera.offsetHeight;
		

		var scaleX = ((mCanvasCameraDisplay.scrollWidth * 1.0) / (mCanvasCameraQQVGA.scrollWidth * 1.0));
		var scaleY = ((mCanvasCameraDisplay.scrollHeight * 1.0) / (mCanvasCameraQQVGA.scrollHeight * 1.0));
		mCtxCameraDisplay.scale(scaleX,scaleY);
		log("canvas scale: " + scaleX + "x" + scaleY);

	}

	mCtxCameraDisplay.fillText("Camera", 0, 0);
}

//https://github.com/ThingPulse/minigrafx/issues/8
function display(pixels, pixelcount) {

	if (mDebugCamera) 
		log("");

	var i = 0;
	var ln = 0;

	for (y = 0; y < mYRes; y++) {
		for (x = 0; x < mXRes; x++) {
			i = (y * mXRes + x) << 1;
			pixel16 = (0xffff & pixels[i]) | ((0xffff & pixels[i + 1]) << 8);
			mImgData.data[ln + 0] = ((((pixel16 >> 11) & 0x1F) * 527) + 23) >> 6;
			mImgData.data[ln + 1] = ((((pixel16 >> 5) & 0x3F) * 259) + 33) >> 6;
			mImgData.data[ln + 2] = (((pixel16 & 0x1F) * 527) + 23) >> 6;
			mImgData.data[ln + 3] = (0xFFFFFFFF) & 255;
			ln += 4;
		}
	}

	mCtxCameraQQVGA.putImageData(mImgData, 0, 0);
	mCtxCameraDisplay.drawImage(mCanvasCameraQQVGA, 0, 0, mCanvasCameraQQVGA.width, mCanvasCameraQQVGA.height);

	// // Request next frame

	// sendWS("camframe");
}

function reset() {

	log("");
	initCanvas();
}

function reconnect() {
	log("");
	init();
	initCanvas();
	capture();
}

function capture() {

	log("");
	if (mWS.readyState != 1) {
		log("mWS.readyState " + mWS.readyState);    
		return;
	}

	reset();

	mImgData = mCtxCameraQQVGA.createImageData(mCanvasCameraQQVGA.width, mCanvasCameraQQVGA.height);
	for (var i = 0; i < mImgData.data.length; i += 4) {
		mImgData.data[i + 0] = 0xCC;
		mImgData.data[i + 1] = 0xCC;
		mImgData.data[i + 2] = 0xCC;
		mImgData.data[i + 3] = 255;
	}

	mCtxCameraQQVGA.putImageData(mImgData, mCanvasCameraQQVGA.width, mCanvasCameraQQVGA.height);

	setText("spanCamera", "Camera initializing ...");

	sendWS("caminit"); // Request camera init
}

////// Toolbar

function toolbarAction(action) {

	log("action: "+ action);

	switch (action) {

		case "fast": // Speed mode - fast

			if (mSpeedMode > 1) {
				if (mConnected) {
					sendWS('caropt:fast');
				} else { // Design tests
					mSpeedMode--;
					initMotorGauges(100/mSpeedMode);
				}
			}
	
			break;
	
		case "slow": // Speed mode - slow
			
			if (mSpeedMode < 10) {
				if (mConnected) {
					sendWS('caropt:slow');
				} else { // Design tests
					mSpeedMode++;
					initMotorGauges(100/mSpeedMode);
				}
			}
		
			break;
	
		case "lights": // Lights on/off
			
			if (mConnected) {
				sendWS('carlights');
			}

			break;
	
		case "reset": // Reset page and robot

			// Reset robot

			if (mConnected) {
				sendWS('reset');
			}

			// Reload page

			location.reload(true);
	
			break;
	
		case "exit": // Exit 

			// Reset robot

			if (mConnected) {
				sendWS('reset');
			}

			// Exit from page

			window.location.href = "about:blank";

			break;
	
		default:
			break;
	}
}
////// Utility

function setText(id, value) {
	
	// Set element text 
	
	var elem = document.getElementById(id);
	if (elem != null) {
		elem.innerText = value;
	}
}

function setHtml(id, value) {
	
	// Set element html 
	
	var elem = document.getElementById(id);
	if (elem != null) {
		elem.innerHTML = value;
	}
}

function log(message) {

	// Log on console

	try {

		var funcName = "";
		if (log.caller && log.caller.name) {
			funcName = log.caller.name;
		}
		if (message != "") {
			if (funcName != "") {
				console.log(funcName + ": " + message);
			} else {
				console.log(message);
			}
		} else if (funcName != "") {
			console.log(funcName);
		}
	
	} catch (error) {

		console.log("undef: " + message);
	}
}

// End