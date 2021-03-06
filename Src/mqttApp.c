/*
 * mqttApp.c
 *
 *  Created on: 23. 10. 2013
 *      Author: hp
 */


#include "myqttApp.h"
#include "lwip/ip_addr.h"

#include "mymqtt.h"

Mqtt mqtt;

#include <string.h>


void mqttAppMsgReceived(Mqtt *this, uint8_t *topic, uint8_t topicLen, uint8_t *data, uint32_t dataLen)
{
	uint8_t strTopic[topicLen + 1];
	memcpy(strTopic, topic, topicLen);
	strTopic[topicLen] = '\0';

	uint8_t strData[dataLen + 1];
	memcpy(strData, data, dataLen);
	strData[dataLen] = '\0';

	printf("mqtt Topic: %s, Data: %s", strTopic, strData);

}

void mqttAppSend()
{
    uint8_t flag = mqttPublish(&mqtt, "/presence", "Hello, here is mbed!");
}

void mqttAppInit()
{
	ip4_addr_t ip_addr;
	IP4_ADDR(&ip_addr, 192,168,0,100);

	mqttInit(&mqtt, ip_addr, 1883, &mqttAppMsgReceived, "MQTT_FX_Client");
}

void mqttAppConnect()
{
	uint32_t flag;

	mqtt.autoConnect = 0;

    flag = mqttConnect(&mqtt);
}

void mqttAppPublish(char *topic, char *data)
{
	mqttPublish(&mqtt, topic, data);
}

void mqttAppDisconnect()
{
	mqttDisconnectForced(&mqtt);
}


void mqttAppHandle()
{
	mqttLive(&mqtt);
}

void mqttApp()
{
	mqttAppInit();
	mqttAppConnect();
	//mqttAppSend();
	//mqttSubscribe(&mqtt,"mqtt");
}
