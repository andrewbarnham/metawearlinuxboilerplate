#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include "gattlib.h"
#include "metawear/core/metawearboard.h"
#include "metawear/peripheral/led.h"
#include "metawear/core/datasignal.h"
#include "metawear/core/settings.h"
#include "metawear/sensor/switch.h"
#include "metawear/processor/rss.h"
#include "metawear/sensor/accelerometer.h"
#include "metawear/core/types.h"
#include "metawear/sensor/gyro_bmi160.h"

static MblMwMetaWearBoard* board;
static gatt_connection_t* gatt_connection;
static gattlib_characteristic_t* characteristics;
static int services_count, characteristics_count;
static	MblMwLedPattern pattern;

static bt_uuid_t battery = { };

typedef struct notify_buffer {
	uint8_t* data;
	size_t length;
	struct notify_buffer* next;
} notify_buffer ;

static struct {
	int 		init;
	pthread_t 	t;
	pthread_mutex_t m;
	pthread_cond_t  c;
	struct notify_buffer *head;
	struct notify_buffer *tail;
} notify_head;

static struct {
	int 		status;
	pthread_mutex_t m;
	pthread_cond_t  c;
} status = {0, PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};


static void setStatus(int in)
{
	pthread_mutex_lock(&status.m);
	status.status=in;
	pthread_cond_signal(&status.c);
	pthread_mutex_unlock(&status.m);
}

static int getStatus()
{
	int result;
	pthread_mutex_lock(&status.m);
	result=status.status;
	pthread_mutex_unlock(&status.m);
	return result;
}

static void waitStatus(int in)
{
	pthread_mutex_lock(&status.m);
	while ( status.status!=in ) {
		pthread_cond_wait(&status.c,&status.m);
	}
	pthread_mutex_unlock(&status.m);
}

static void * notify_loop()
{
	while ( 1) {
		pthread_mutex_lock(&notify_head.m);
		notify_buffer *v;
		v = notify_head.head;
		if (v==NULL) {
			pthread_cond_wait(&notify_head.c,&notify_head.m);
			pthread_mutex_unlock(&notify_head.m);		
			continue;
		}
		notify_head.head=notify_head.head->next;
		pthread_mutex_unlock(&notify_head.m);		

		//printf("notify something\n");
		mbl_mw_metawearboard_notify_char_changed(board,v->data,v->length);


		free(v->data);
		free(v);
	}
}

static void notification_handler(uint16_t handle, const uint8_t* data, size_t data_length, void* user_data)
{
	//printf("Got D something for %hi l:%i %lx\n",handle,data_length,pthread_self());

	notify_buffer *nb;
	nb=malloc(sizeof(notify_buffer));
	nb->length=data_length;
	nb->data=malloc(data_length);
	nb->next=NULL;
	memcpy(nb->data,data,data_length);

	if (notify_head.init==0) {	
		notify_head.init=1;
		pthread_mutex_init(&notify_head.m,NULL);
		pthread_cond_init(&notify_head.c,NULL);
		pthread_create(&notify_head.t,NULL,&notify_loop,NULL);
	}

	pthread_mutex_lock(&notify_head.m);
	
	if (notify_head.head == NULL) {
		notify_head.head = nb;
	} else {
		notify_head.tail->next =nb;
	}
	notify_head.tail = nb;
		
	pthread_cond_signal(&notify_head.c);
	pthread_mutex_unlock(&notify_head.m);
}

static void connect_cb(gatt_connection_t* connection)
{
        if (connection != NULL) {
                gatt_connection = connection;
                gattlib_register_notification(connection, notification_handler, NULL);
        }
	setStatus(1);
}

static int uuid_match(const bt_uuid_t *uuid,const MblMwGattChar *characteristic)
{

        bt_uuid_t tmp;
	unsigned long 	*hi;
	unsigned long   *lo;
        const uint8_t *data;

        /* Convert to 128 Bit format */
        bt_uuid_to_uuid128(uuid, &tmp);
        data = (uint8_t *) &tmp.value.u128;

	hi=(void *)&data[0];
	lo=(void *)&data[8];

	if (be64toh(*hi)==characteristic->uuid_high && be64toh(*lo)==characteristic->uuid_low) {
		return 1;
	}

       return 0;
}


static void write_gatt_char(const void* caller, const MblMwGattChar *characteristic, const uint8_t *value, uint8_t length) {
		
	int i;
        for (i = 0; i < characteristics_count; i++) {
		if (uuid_match(&characteristics[i].uuid,characteristic)) {
			
			if (getStatus()<2) {
				printf(".");
				fflush(stdout);
			}

			/*
			printf("Write: %lx - %lx %i H:%i\n",characteristic->uuid_high,characteristic->uuid_low,length,characteristics[i].value_handle);
			int s;
			printf("   ");
			for (s=0;s<length;s++) {
				printf("%02X", value[s]);
			}
			printf("\n");
			*/
			int r =gattlib_write_char_by_handle(gatt_connection,characteristics[i].value_handle,(void *)value,length);
			return;
		}
	}

	printf("Don't know how to write to %lx - %lx\n",characteristic->uuid_high,characteristic->uuid_low);
}

static void read_gatt_char(const void* caller, const MblMwGattChar *characteristic) { 
	int i;
	//printf("In read %lx\n",pthread_self());
        for (i = 0; i < characteristics_count; i++) {

		if (uuid_match(&characteristics[i].uuid,characteristic)) {
		        
			uint8_t buffer[100];

			int len=gattlib_read_char_by_uuid(
				gatt_connection,
				&characteristics[i].uuid,buffer,sizeof(buffer));

			//printf("Read: %lx - %lx %i\n",characteristic->uuid_high,characteristic->uuid_low,len);
			mbl_mw_metawearboard_char_read(board,characteristic,buffer,5);
			return;
			
		}
	}
	printf("Don't know how to read from %lx - %lx\n",characteristic->uuid_high,characteristic->uuid_low);
}

static void  init(MblMwMetaWearBoard* board, int32_t status) {
	printf("\n");
	    if (status) {
	        printf("Error initializing board: %d\n", status);
	    } else {
	        printf("Board initialized\n");
		setStatus(2);
	    }
}

static void switch_handler(const MblMwData* data) 
{
     if (*((uint32_t*)data->value)) {
	   printf ("Switch Pressed\n");
    } else {
	   printf ("Switch Released\n");
    }
}

static int accel_count;
static time_t last_accel;

static void accel_handler(const MblMwData* data) 
{
	MblMwCartesianFloat *acceleration = (MblMwCartesianFloat*) data->value;

	time_t n = time(NULL);
	if (n!=last_accel) {
		printf("ACCL S/R: %i %f %f %f\n",accel_count,acceleration->x, acceleration->y, acceleration->z);
		last_accel=n;
		accel_count=0;
	}
	accel_count++;

        //
        //printf("(%.3fg, %.3fg, %.3fg)\n", );
}

static int gyro_count;
static time_t last_gyro;


static void gyro_handler(const MblMwData* data) 
{
        MblMwCartesianFloat *acceleration = (MblMwCartesianFloat*) data->value;

	time_t n = time(NULL);
	if (n!=last_gyro) {
		printf("GYRO S/R: %i %f %f %f\n",gyro_count,acceleration->x, acceleration->y, acceleration->z);
		last_gyro=n;
		gyro_count=0;
	}
	gyro_count++;
	

//        printf("(%.3fdps, %.3fdps, %.3fdps)\n", acceleration->x, acceleration->y, acceleration->z);
}

static void battery_handler(const MblMwData* data)
{
	MblMwBatteryState *state = (MblMwBatteryState*) data->value;
        printf("{voltage: %dmV, charge: %d}\n", state->voltage, state->charge);

}

int main(int argc, char *argv[]) {
	int dev_id;
	dev_id = hci_get_route(NULL);
	int device_desc = hci_open_dev(dev_id);

        char* addr = argv[1];
        gattlib_primary_service_t* services;
        char uuid_str[MAX_LEN_UUID_STR + 1];
        int ret, i;

        printf("------------START %s ---------------\n", addr);
        //gatt_connection = gattlib_connect(NULL, addr, BDADDR_LE_PUBLIC, BT_IO_SEC_LOW, 0, 0);
	gattlib_connect_async(NULL, addr, BDADDR_LE_RANDOM, BT_IO_SEC_LOW, 0, 0,connect_cb);
	waitStatus(1);
        if (gatt_connection == NULL) {
                fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 0;
	}
        puts("Succeeded to connect to the bluetooth device with random address.");
	

        ret = gattlib_discover_primary(gatt_connection, &services, &services_count);
        if (ret != 0) {
                fprintf(stderr, "Fail to discover primary services.\n");
                return 0;
        }

        for (i = 0; i < services_count; i++) {
                gattlib_uuid_to_string(&services[i].uuid, uuid_str, sizeof(uuid_str));
                printf("service[%d] start_handle:%02x end_handle:%02x uuid:%s\n", i,
                                services[i].attr_handle_start, services[i].attr_handle_end,
                                uuid_str);
        }

        ret = gattlib_discover_char(gatt_connection, &characteristics, &characteristics_count);
        if (ret != 0) {
                fprintf(stderr, "Fail to discover characteristics.\n");
                return 0;
        }
        for (i = 0; i < characteristics_count; i++) {
                gattlib_uuid_to_string(&characteristics[i].uuid, uuid_str, sizeof(uuid_str));

                printf("characteristic[%d] properties:%02x value_handle:%04x uuid:%s\n", i,
                                characteristics[i].properties, characteristics[i].value_handle,
                                uuid_str);

		/*
		if (uuid_match(&characteristics[i].uuid,&METAWEAR_SERVICE_NOTIFY_CHAR)) {
			notify=&characteristics[i].uuid;
		}
		*/
        }

	// char-write-req 0020 0100
	char myArray[] = { 0x01, 0x00 };
	gattlib_write_char_by_handle(gatt_connection,(uint16_t)0x20, myArray,2);

	uint8_t buffer[100];
	bt_uuid_t batt;
	bt_string_to_uuid(&batt,"00002a19-0000-1000-8000-00805f9b34fb");
	int len=gattlib_read_char_by_uuid(gatt_connection,&batt,buffer,sizeof(buffer));
	printf("BATTERY :%i\n",buffer[0]);




	MblMwBtleConnection btle_conn = { write_gatt_char, read_gatt_char };
	board = mbl_mw_metawearboard_create(&btle_conn);
	printf("Initializing:");
	fflush(stdout);
	mbl_mw_metawearboard_initialize(board,&init);
	waitStatus(2);
	sleep(1);
	if (mbl_mw_metawearboard_is_initialized(board)) {
	        printf("LED %i\n",mbl_mw_metawearboard_lookup_module(board,MBL_MW_MODULE_LED));
		
		mbl_mw_led_load_preset_pattern(&pattern, MBL_MW_LED_PRESET_BLINK);	
		mbl_mw_led_write_pattern(board, &pattern, MBL_MW_LED_COLOR_BLUE);
		mbl_mw_led_play(board);

		MblMwDataSignal *switch_signal = mbl_mw_switch_get_state_data_signal(board);
		mbl_mw_datasignal_subscribe(switch_signal, &switch_handler);	

		//MblMwDataSignal *battery_signal = mbl_mw_settings_get_battery_state_data_signal(board);
    		//mbl_mw_datasignal_subscribe(battery_signal,&battery_handler);
       	        //mbl_mw_datasignal_read(battery_signal);	

		MblMwDataSignal *acc_signal = mbl_mw_acc_get_acceleration_data_signal(board);
		mbl_mw_datasignal_subscribe(acc_signal, &accel_handler);	
		mbl_mw_acc_set_odr(board, 200.f);	
  		mbl_mw_acc_enable_acceleration_sampling(board);
 	   	mbl_mw_acc_start(board);

		MblMwDataSignal *gyro_signal = mbl_mw_gyro_bmi160_get_rotation_data_signal(board);
		mbl_mw_datasignal_subscribe(gyro_signal,&gyro_handler);
		mbl_mw_gyro_bmi160_set_odr(board, 200.f);
		mbl_mw_gyro_bmi160_enable_rotation_sampling(board);
		mbl_mw_gyro_bmi160_start(board);


		sleep(30);

		mbl_mw_datasignal_unsubscribe(switch_signal);

		mbl_mw_led_stop(board);
	}


	mbl_mw_metawearboard_free(board);
        free(services);
        free(characteristics);
        gattlib_disconnect(gatt_connection);
        printf("------------FINISH %s ---------------\n", addr);
}
	

