#include "lwip/apps/mqtt.h"
#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/mem.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include <string.h>
#include <time.h>

static int inpub_id;
void example_do_connect(mqtt_client_t *client);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  printf("Incoming publish payload with length %d, flags %u\n", len, (unsigned int)flags);

  if(flags & MQTT_DATA_FLAG_LAST) {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0) {
      /* Don't trust the publisher, check zero termination */
      if(data[len-1] == 0) {
        printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
      }
    } else if(inpub_id == 1) {
    	printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
      /* Call an 'A' function... */
    } else {
    	printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
    }
  } else {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, "print_payload") == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* All topics starting with 'A' might be handled at the same way */
    inpub_id = 1;
  } else {
    /* For all other topics */
    inpub_id = 2;
  }
}

static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}

static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n", result);
}

void example_publish(mqtt_client_t *client, void *arg)
{
  const char *pub_payload= "PubSubHubLubJub";
  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  err = mqtt_publish(client, "pub_topic", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, "subtopic", 1, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n", err);
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);

    /* Its more nice to be connected, so try to reconnect */
    example_do_connect(client);
  }
}

void example_do_connect(mqtt_client_t *client)
{
  struct mqtt_connect_client_info_t ci;
  err_t err;
  ip4_addr_t ip_addr;
  char arr[] = "pub_topic";
  /* Setup an empty client info structure */
  memset(&ci, 0, sizeof(ci));
  char username[] = "solace-cloud-client";
  char password[] = "umcpfrkgupnq2vtvgpp1ght923";

  /* Minimal amount of information required is client identifier, so set it here */
  ci.client_id = "pkumarasamy@ymail.com";
  ci.will_msg = arr;
  ci.will_qos = 0;
  ci.will_topic = arr;
  ci.client_user = username;
  //printf("\r\n User name = %s\n",ci.client_user);
  ci.client_pass = password;
  //ci.keep_alive = 10;

  //ip_addr_t ip_addr;
  /* initliaze IP addresses to be used */
  IP4_ADDR(&ip_addr,   35,193,101,112 );

  /* Initiate client and connect to server, if this fails immediat	ely an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting
     to establish a connection with the server.
     For now MQTT version 3.1.1 is always used */

  err = mqtt_client_connect(client, &ip_addr, MQTT_PORT, mqtt_connection_cb, NULL, &ci);

  /* For now just print the result code if something goes wrong */
  if(err != ERR_OK) {
    printf("mqtt_connect return %d\n", err);
  }
}


void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Stroing start time
    clock_t start_time = clock();

    // looping till required time is not acheived
    while (clock() < start_time + milli_seconds)
        ;
}
mqtt_client_t *client;
void  mqtt_init()
{
	if ( client == NULL )
		client = mqtt_client_new();
    printf("mqtt_init\n");

    if(client != NULL)
	{
	    //example_do_connect(client);

	    //while(1)
	      {
		    	  /* while connected, publish every second */
	    	  if(mqtt_client_is_connected(client))
	    	  {
	    		  //sys_untimeout(cyclic_timer,NULL);
	    		  printf("Connected\n");
	    		  example_publish(client, NULL);
	    		  sys_timeout(10000, cyclic_timer,NULL);
	    	  }
	    	  else
	    	  {
	    		  /* Connect to server */
	    		  example_do_connect(client);
	    		  sys_timeout(10000, cyclic_timer,NULL);
	    	  }
	      }
	 }
}

void cyclic_timer(void *arg)
{
  mqtt_init();
}
