//########data
let data_tx = pins.createBuffer(38);
let gait_mode = 0; //robot status
let rc_spd_cmd_X = 0.00 //x_speed
let rc_spd_cmd_y = 0.00 //y_speed
let rc_att_rate_cmd = 0.00 // Turn to speed
let rc_spd_cmd_z = 0.00 //mobile speed
let rc_pos_cmd = 0.00 //height
let rc_att_cmd_x = 0.00 //Pitch
let rc_att_cmd_y = 0.00 //Side swing
let rc_att_cmd = 0.00 //Heading
let robot_mode = 0
let robot_mode_1 = 0
let state = 0

//########SPI
let SSLen = 50
let InfoTemp = pins.createBuffer(SSLen)
let ToSlaveBuf = pins.createBuffer(SSLen)
let SfoCnt = 0
let DaHeader = 0x2B
let DaTail = 0xEE
let usb_send_cnt = 0
let cnt = 0

//########Steering gear||舵机
let ToSlaveBuf_1 = pins.createBuffer(SSLen)
let InfoTemp_1 = pins.createBuffer(SSLen)
let usb_send_cnt_1 = 0
let SfoCnt_1 = 0
let DaHeader_1 = 0x2B
let DaTail_1 = 0xEE