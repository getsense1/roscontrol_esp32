#include "ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "esp_chatter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define _DIC_ROWS   (4)

static const char *TAG4 = "TCP->";
static const float _dict[_DIC_ROWS][4]={ {0.0, 0.0,0.0,0.0},
                                         {0.1, 1.1,1.2,1.3},
                                         {0.2, 2.1,2.2,2.3},
                                         {0.3, 3.1,3.2,3.3}

};

static float cmd=0.0;
static struct _pve_state{
            float pos;
            float vel;
            float eff;
} _state;


void _chat_recv_cb( const std_msgs::Float64& _msg){
    ESP_LOGI(TAG4, "cmd recieved: %f",_msg.data);
    cmd=_msg.data;
}

ros::NodeHandle nh;

std_msgs::Float64MultiArray _msg;
ros::Publisher chatter("cmd_heater_rx", &_msg);
ros::Subscriber<std_msgs::Float64> chatter_recv("cmd_heater_tx", &_chat_recv_cb );





void rosserial_setup()
{
  _msg.data_length=3;
  _msg.data=&_state.pos;
  // Initialize ROS
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(chatter_recv);

  _state.pos= _dict[0][1];
  _state.eff= _dict[0][2];
  _state.vel= _dict[0][3];

}

void rosserial_publish()
{
unsigned char _i0;
float _i1;
/*
cmd is key of dictionary {pos,vel,eff}
found {pos,vel,eff} send to ROS_Control node
*/
  while(1) {
    ESP_LOGI(TAG4, "L0 here cmd->: %f",cmd);
    for(_i0=0;_i0<_DIC_ROWS;_i0++){
        _i1=fabsf(cmd-_dict[_i0][0]);
        if(_i1<0.1){
            ESP_LOGI(TAG4, "L0 here: %d,%f",_i0,_i1);
            _state.pos= _dict[_i0][1];
            _state.eff= _dict[_i0][2];
            _state.vel= _dict[_i0][3];
        }
    }

    // Send the message
    chatter.publish(&_msg);
    if(nh.spinOnce()<0){
        ESP_LOGE(TAG4, "spinOnce -> ERROR");
        continue;
    }
    ESP_LOGI(TAG4, "state sent: %f,%f,%f",_state.pos,_state.eff,_state.vel);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}
