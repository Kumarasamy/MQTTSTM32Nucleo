/*
 * mqttApp.h
 *
 *  Created on: 23. 10. 2013
 *      Author: hp
 */

#ifndef MYQTTAPP_H_
#define MYQTTAPP_H_

void mqttAppInit();
void mqttAppConnect();
void mqttAppSend();
void mqttAppHandle();
void mqttAppDisconnect();
void mqttAppPublish(char *topic, char *data);
void mqttApp();
#endif /* MYQTTAPP_H_ */
