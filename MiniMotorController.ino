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

KinematicController kc(&m1,&m2,1,-1,228,75,64*50);

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
	}

}


int responseState;

void receiveEvent(int howMany){
	byte command = Wire.read();
	uint16_t arg[16];
	switch (command) {
		case COMMAND_HEARTBEAT:
			lastHeartbeat = millis();
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
		default:
		break;
	}
	while (Wire.available() > 0){
		Wire.read();
		Serial.println("too many");
		digitalWrite(13,HIGH);
	}
}

void requestEvent(){
	if (responseState == COMMAND_REPORTSTANDBY) {
		byte resp = kc.isStandby()?0x01:0x00;
		Wire.write(resp);
		responseState = 0x00;
	} else if (responseState == COMMAND_REPORTENCODER){
		byte resp[8];
		int32_t fwd = kc.getOdometryForward();
		int32_t ccw = kc.getOdometryCCW();
		resp[0] = fwd;
		resp[1] = fwd >> 8;
		resp[2] = fwd >> 16;
		resp[3] = fwd >> 24;
		resp[4] = ccw;
		resp[5] = ccw >> 8;
		resp[6] = ccw >> 16;
		resp[7] = ccw >> 24;
		Wire.write(resp,8);
		responseState = 0x00;
	} else {
		digitalWrite(13,HIGH);
	}

}