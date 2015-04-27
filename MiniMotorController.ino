#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <PWM.h>
#include <Wire.h>
#include <MemoryFree.h>
#include <Encoder.h>
#include <RegulatedMotor.h>;
#include <KinematicController.h>
#include "MiniMotorControllerDefinitions.h"

Encoder enc1(2,4);
Encoder enc2(3,5);

long encoder1;
long encoder2;

RegulatedMotor m1(&encoder1,7,6,10);
RegulatedMotor m2(&encoder2,8,11,9);

//KinematicController kc(&m1,&m2,1,-1,225,75,64*50);
KinematicController kc(&m1,&m2,1,-1,226,75.485625,64*50);

void setup(){
	Serial.begin(115200);
	Wire.begin(MOTOR_CONTROLLER_ADDRESS);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);
	pinMode(8,OUTPUT);
	pinMode(9,OUTPUT);
	pinMode(10,OUTPUT);
	pinMode(11,OUTPUT);
	pinMode(13,OUTPUT);
	digitalWrite(13,LOW);

	InitTimersSafe();      
	SetPinFrequencySafe(10, 150);
	SetPinFrequencySafe(9, 150);

	m1.setSampleTime(20000);
	m2.setSampleTime(20000);
	m1.setPID(0.19,0.0085,0.02,0.0);
	m2.setPID(0.19,0.0085,0.02,0.0);

	kc.setAcceleration(1000,1000,1000,1000);
	kc.coast();
}

long lastHeartbeat = 0;

void loop(){
	encoder1 = enc1.read();
	encoder2 = enc2.read();
	kc.run();
	m1.run();
	m2.run();

	if ( millis()-lastHeartbeat > 5000 ){
		kc.brake();
		digitalWrite(13,HIGH);
	} else {
		digitalWrite(13,LOW);
	}

}


int responseState;

void receiveEvent(int howMany){
	lastHeartbeat = millis();
	byte command = Wire.read();
	uint16_t arg[16];
	switch (command) {
		case COMMAND_HEARTBEAT:
		break;
		case COMMAND_SETACCELERATION:
			for (int i = 0; i < 4; i++){
				arg[i] = Wire.read();
				arg[i] |= Wire.read() << 8;
			}
			kc.setAcceleration(arg[0],arg[1],arg[2],arg[3]);
		break;
		case COMMAND_GOVELOCITY:
			for (int i = 0; i < 2; i++){
				arg[i] = Wire.read();
				arg[i] |= Wire.read() << 8;
			}
			kc.goVelocity(arg[0],arg[1]);
		break;
		case COMMAND_BRAKE:
			kc.brake();
		break;
		case COMMAND_COAST:
			kc.coast();
		break;
		case COMMAND_REPORTSTANDBY:
			responseState = COMMAND_REPORTSTANDBY;
		break;
		case COMMAND_REPORTENCODER:
			responseState = COMMAND_REPORTENCODER;
		break;
		case COMMAND_CALIBRATE:
			for (int i = 0; i < 2; i++){
				arg[i] = Wire.read();
				arg[i] |= Wire.read() << 8;
			}
			kc.calibrate(arg[0],arg[1]);
		break;
		case COMMAND_GETGLOBALPOSITION:
			responseState = COMMAND_GETGLOBALPOSITION;
		break;
		/*
		case COMMAND_PID:
			float floatArgs[4];
			for (int i = 0; i < 4; i++){
				floatArgs[i] = Wire.read();
				floatArgs[i] |= Wire.read() << 8UL;
				floatArgs[i] |= Wire.read() << 16UL;
				floatArgs[i] |= Wire.read() << 24UL;
			}
			m1.setPID(floatArgs[0],floatArgs[1],floatArgs[2],floatArgs[3]);
			m2.setPID(floatArgs[0],floatArgs[1],floatArgs[2],floatArgs[3]);
		break;
		*/
		default:
		break;
	}
	while (Wire.available() > 0){
		Wire.read();
		digitalWrite(13,HIGH);
	}
}

void requestEvent(){
	if (responseState == COMMAND_REPORTSTANDBY) {
		byte resp[1];
		resp[0] = kc.isStandby()?0x01:0x00;
		Wire.write(resp,1);
		responseState = 0x00;
	} else if (responseState == COMMAND_REPORTENCODER){
		byte resp[8];
		int32_t fwd = kc.getOdometryForward();
		int32_t ccw = kc.getOdometryCCW();
		resp[0] = (int32_t) fwd;
		resp[1] = (int32_t) fwd >> 8UL;
		resp[2] = (int32_t) fwd >> 16UL;
		resp[3] = (int32_t) fwd >> 24UL;
		resp[4] = (int32_t) ccw;
		resp[5] = (int32_t) ccw >> 8UL;
		resp[6] = (int32_t) ccw >> 16UL;
		resp[7] = (int32_t) ccw >> 24UL;
		Wire.write(resp,8);
		responseState = 0x00;
	} else if (responseState == COMMAND_GETGLOBALPOSITION) {
		byte resp[8];
		int32_t x;
		int32_t y;
		kc.getGlobalPosition(&x, &y);
		resp[0] = (int32_t) x;
		resp[1] = (int32_t) x >> 8UL;
		resp[2] = (int32_t) x >> 16UL;
		resp[3] = (int32_t) x >> 24UL;
		resp[4] = (int32_t) y;
		resp[5] = (int32_t) y >> 8UL;
		resp[6] = (int32_t) y >> 16UL;
		resp[7] = (int32_t) y >> 24UL;
		Wire.write(resp,8);
		responseState = 0x00;		
	} else {

	}

}