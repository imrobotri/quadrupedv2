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

//########Joint angle control||关节角度控制
let ToSlaveBuf_2 = pins.createBuffer(SSLen)
let InfoTemp_2 = pins.createBuffer(SSLen)
let usb_send_cnt_2 = 0
let SfoCnt_2 = 0
let DaHeader_2 = 0x2B
let DaTail_2 = 0xEE
let FL_d = 45.0
let FL_x = 90.0
let FL_c = 0.0
let FR_d = 45.0
let FR_x = 90.0
let FR_c = 0.0
let HL_d = 45.0
let HL_x = 90.0
let HL_c = 0.0
let HR_d = 45.0
let HR_x = 90.0
let HR_c = 0.0

//########Image Identification||图像识别
//------------definition--------------
let Identify_TX = pins.createBuffer(9)
let Identify_RX = pins.createBuffer(50)
let cnt_p = 0

//识别设置
let TestTX = pins.createBuffer(7)
let FunID = 0x00
let ColID = 0x00
let ShaID = 0x00
let ShaColID = 0x00
let FrameHeader = 0x00
let DataID = 0x00

//Color
let Color_ID = 0x00

//Shapes
let Shapes_ID = 0x00

//QR code
let Identify_x = 0x00, Identify_y = 0x00, Identify_z = 0x00
let Identify_Flip_x = 0x00, Identify_Flip_y = 0x00, Identify_Flip_z = 0x00
let Identify_status = 0x00, Identify_pattern = 0x00

//mall ball
let Ball_status = 0x00  //status
let Ball_X = 0x00, Ball_Y = 0x00 //x-axis, y-axis
let Ball_W = 0x00, Ball_H = 0x00 //Width Height
let Ball_pixels = 0x00  //Number of pixels

//Line inspection
let Line_detect = 0x00 //Detect
let Line_effect = 0x00 //The effect of the identification line
let Line_angle = 0x00 //angle
let Line_position = 0x00 //position
let s = 0

let CRC_L = 0x00
let CRC_H = 0x00

let CRC_tx_L = 0x00
let CRC_tx_H = 0x00

let CRC_tx_L1 = 0x00
let CRC_tx_H1 = 0x00

let Function_s = 0      //Function selection (1: two-dimensional code 2: small ball 3: line patrol)
let Function_c = 0x00   //function code

//PID 
let Kp_P = 0.260  //P_Position adjustment parameters
let Kp_A = 0.350  //P_ Angle adjustment parameters
let Ki_P = 0.0000  //D_Position adjustment parameters
let Ki_A = 0.0000 //D_Angle adjustment parameters
let Kd_P = 0.0000  //D_Position adjustment parameters
let Kd_A = 0.0000  //D_Angle adjustment parameters

let error_P = 0.000 // Position error
let error_A = 0.000 // Angle error

let _error_P = 0.000 // Position error
let _error_A = 0.000 // Angle error

let integral_P = 0.000 // Accumulated value of position error
let integral_A = 0.000 // Accumulated value of angle error
let Ki_ovP = 0.000       //Ki output value
let Ki_ovA = 0.000       //Ki output value

let derivative_P = 0.000 //Error difference
let prev_error_P = 0.000 //Store last error
let derivative_A = 0.000 //Error difference
let prev_error_A = 0.000 //Store last error

let speed_P = 0.000 //Movement speed output
let speed_A = 0.000 //Angle speed output

let time = 0.00


//########Speech Recognition||语音识别
let get_data = 0x00 //retrieve data
let voice_speed = 7 //speed






//########SPI_int||SPI初始化
function SPI_Init() {
    pins.digitalWritePin(DigitalPin.P16, 1)
    pins.digitalWritePin(DigitalPin.P12, 1)
    pins.spiPins(DigitalPin.P15, DigitalPin.P14, DigitalPin.P13)
    pins.spiFrequency(1000000)
    led.enable(false)
    //serial.writeNumber(1)
}

//########SPI data transmission/reception||SPI数据发送/接收
function SPI_Send() {
    if (state == 1) {
        SPICom_Walk()
        pins.digitalWritePin(DigitalPin.P16, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        for (let i = 0; i < 200; i++);
        for (let i = 0; i < SSLen; i++) {
            InfoTemp[i] = pins.spiWrite(ToSlaveBuf[i])
        }
        pins.digitalWritePin(DigitalPin.P12, 1)
        pins.digitalWritePin(DigitalPin.P16, 1)
        //serial.writeBuffer(InfoTemp)
        //serial.writeBuffer(ToSlaveBuf)
        SPI_unpacking()
        basic.pause(1)
    }
}
//########Control data||控制数据
function SPICom_Walk() {
    usb_send_cnt = 0
    let cnt_reg = 0
    let sum = 0
    ToSlaveBuf[usb_send_cnt++] = DaHeader; //头
    ToSlaveBuf[usb_send_cnt++] = SSLen - 2; //固定长度
    ToSlaveBuf[usb_send_cnt++] = 1;  //功能码

    ToSlaveBuf[usb_send_cnt++] = gait_mode;
    get_float_hex(rc_spd_cmd_X)
    get_float_hex(rc_spd_cmd_y)
    get_float_hex(rc_att_rate_cmd)
    get_float_hex(rc_spd_cmd_z)
    get_float_hex(rc_pos_cmd) //0.1
    get_float_hex(rc_att_cmd_x)
    get_float_hex(rc_att_cmd_y)
    get_float_hex(rc_att_cmd)

    ToSlaveBuf[SSLen - 1] = DaTail;
}

//########Data analysis||数据解析
function SPI_unpacking() {
    cnt = 0
    if (InfoTemp[0] == 0x2B && InfoTemp[2] == 0x80)
        robot_mode = InfoTemp[3]
    //serial.writeNumber(robot_mode)
}

//########Stand in place||原地站立
function Standing() {
    if (robot_mode == 1)
        return
    gait_mode = 5
    while (1) {
        SPI_Send()
        if (robot_mode == 1 || robot_mode == 0x02) {
            gait_mode = 4
            SPI_Send()
            return;
        }
    }
}

//########Servo SPI data transmission||舵机SPI 数据发送
function SG_SPI_Send() {
    pins.digitalWritePin(DigitalPin.P16, 0)
    pins.digitalWritePin(DigitalPin.P12, 0)
    for (let i = 0; i < 200; i++);
    for (let i = 0; i < SSLen; i++) {
        InfoTemp_1[i] = pins.spiWrite(ToSlaveBuf_1[i])
    }
    //serial.writeBuffer(ToSlaveBuf_1)
    pins.digitalWritePin(DigitalPin.P12, 1)
    pins.digitalWritePin(DigitalPin.P16, 1)
    basic.pause(1)
}

//########Joint SPI data transmission||关节SPI 数据发送
function Joint_SPI_Send() {

    Joint_data()
    pins.digitalWritePin(DigitalPin.P16, 0)
    pins.digitalWritePin(DigitalPin.P12, 0)
    for (let i = 0; i < 200; i++);
    for (let i = 0; i < SSLen; i++) {
        InfoTemp_2[i] = pins.spiWrite(ToSlaveBuf[i])
    }
    pins.digitalWritePin(DigitalPin.P12, 1)
    pins.digitalWritePin(DigitalPin.P16, 1)
    //SPI_unpacking()
    basic.pause(1)

    //    }
}

function Joint_data() {
    usb_send_cnt = 0
    let cnt_reg = 0
    let sum = 0
    ToSlaveBuf[usb_send_cnt++] = DaHeader_2; //头
    ToSlaveBuf[usb_send_cnt++] = SSLen - 2; //固定长度
    ToSlaveBuf[usb_send_cnt++] = 0x03;  //功能码

    ToSlaveBuf[usb_send_cnt++] = 0x01;
    get_float_hex(FL_d)
    get_float_hex(FL_x)
    get_float_hex(FL_c)
    get_float_hex(FR_d)
    get_float_hex(FR_x)
    get_float_hex(FR_c)
    get_float_hex(HL_d)
    get_float_hex(HL_x)
    get_float_hex(HL_c)
    get_float_hex(HR_d)
    get_float_hex(HR_x)
    get_float_hex(HR_c)

    ToSlaveBuf[SSLen - 1] = DaTail_2;
}


// //识别功能、颜色开、形状、形状颜色开启
// function IRecognitionSettings() {
//     let cnt = 0
//     let i = 0
//     let sum = 0x00
//     TestTX[cnt++] = FrameHeader
//     TestTX[cnt++] = 0x00
//     TestTX[cnt++] = FunID
//     TestTX[cnt++] = ColID
//     TestTX[cnt++] = ShaID
//     TestTX[cnt++] = ShaColID
//     TestTX[1] = cnt - 2
//     for (i; i < cnt;i++) {
//         sum = sum + TestTX[i]
//     }
//     TestTX[cnt] = sum
//     serial.writeBuffer(TestTX)
//     basic.pause(10)
//  }

// 功能开启
function IRecognitionSettings() {
    let cnt = 0
    let i = 0
    let sum = 0x00
    TestTX[cnt++] = FrameHeader         //帧头
    TestTX[cnt++] = DataID              //数据ID
    TestTX[cnt++] = 0x00                //数据长度
    if (DataID == 0x01) {
        TestTX[cnt++] = FunID           //功能ID
        TestTX[cnt++] = ColID
    }
    else if (DataID == 0x02) {
        TestTX[cnt++] = FunID           //功能ID
        TestTX[cnt++] = ShaID
        TestTX[cnt++] = ColID
    }
    else if (DataID == 0x03) {
        TestTX[cnt++] = FunID           //功能ID
    }
    else if (DataID == 0x04) {
        TestTX[cnt++] = FunID
        TestTX[cnt++] = ColID
        TestTX[cnt++] = ShaColID
    }
    TestTX[2] = cnt - 3                 //计算数据长度 
    for (i; i < cnt; i++) {
        sum = sum + TestTX[i]
    }
    TestTX[cnt] = sum
    serial.writeBuffer(TestTX)
    basic.pause(10)
}

// 功能切换
function IRecognitionToggle() {
    let RXSS = 0X00
    let cnt = 0
    TestTX[cnt++] = 0xBB                   //帧头
    for (let i = 1; i < 7; i++)
        TestTX[cnt++] = 0x00
    serial.writeBuffer(TestTX)
    basic.pause(100)
}


//Data sending（Image Identification）||数据发送（图像识别）
function Identify_send() {
    cnt_p = 0
    Identify_TX[cnt_p++] = 0x00
    Identify_TX[cnt_p++] = 0x01 // ID
    Identify_TX[cnt_p++] = 0x03
    Identify_TX[cnt_p++] = 0x00
    Identify_TX[cnt_p++] = Function_c
    Identify_TX[cnt_p++] = 0x00
    Identify_TX[cnt_p++] = 0x0A
    usMBCRC161(Identify_TX, cnt_p)
    // serial.writeBuffer(Identify_TX)
    Identify_TX[cnt_p++] = CRC_tx_H1
    Identify_TX[cnt_p++] = CRC_tx_L1
    serial.writeBuffer(Identify_TX)
    basic.pause(10)

}
//Data reception（Image Identification）||数据接收（图像识别）
function Identify_receive() {
    //serial.setRxBufferSize(32)
    let position_r = 0
    let sum_r = 0x00
    let length_r = 0
    Identify_RX = serial.readBuffer(0)
    if (Identify_RX[0] == 0x01 && Identify_RX[1] < 0xFF) {
        length_r = Identify_RX[2]
        usMBCRC16(Identify_RX, length_r + 3)
        if (Identify_RX[length_r + 3] == CRC_H && Identify_RX[length_r + 4] == CRC_L) {
            switch (Function_s) {
                case 1: Identify_Color(Identify_RX); break;
                case 2: Identify_collection(Identify_RX); break;
                case 3: Ball_rd(Identify_RX); break;
                case 4: Line_inspection(Identify_RX); break;
                case 5: Identify_Shapes(Identify_RX); break;
                default: return
            }
        }
    }
    return
}

// 颜色识别
function Identify_Color(Identify_RX_1: any) {
    let Identify_RX_2 = pins.createBuffer(10)
    Identify_RX_2 = Identify_RX_1
    let cnt_I = 3
    Color_ID = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])       //颜色ID（1红色、2）
}

// 形状识别
function Identify_Shapes(Identify_RX_1: any) {
    let Identify_RX_2 = pins.createBuffer(50)
    Identify_RX_2 = Identify_RX_1
    let cnt_I = 11
    Shapes_ID = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])       //形状ID（1红色、2）
}



//QR code data acquisition||标签数据采集
function Identify_collection(Identify_RX_1: any) {
    //serial.writeBuffer(Identify_RX_1)
    let Identify_RX_2 = pins.createBuffer(50)
    Identify_RX_2 = Identify_RX_1
    let cnt_I = 3
    Identify_status = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])   //
    // serial.writeNumber(Identify_status)
    Identify_pattern = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])  //
    Identify_x = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])        //
    Identify_y = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])        //
    Identify_z = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])        //
    Identify_Flip_x = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])   //
    Identify_Flip_y = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])   //
    Identify_Flip_z = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])   //
    //serial.writeNumber(Identify_x)
    //basic.showNumber(Identify_pattern)
}

//Ball recognition||识别颜色小球
function Ball_rd(Identify_RX_1: any) {
    let Identify_RX_2 = pins.createBuffer(50)
    Identify_RX_2 = Identify_RX_1
    let cnt_I = 3
    //serial.writeBuffer(Identify_RX_2)
    Ball_status = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])
    Ball_X = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])
    Ball_Y = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])
    Ball_W = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])
    Ball_H = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])
    Ball_pixels = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])
    //serial.writeNumber(Ball_X)
}

//Line patrol identification||巡线识别
function Line_inspection(Identify_RX_1: any) {
    let Identify_RX_2 = pins.createBuffer(50)
    Identify_RX_2 = Identify_RX_1
    let cnt_I = 3
    Line_detect = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++]) //Detect
    Line_effect = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++]) //The effect of the identification line
    Line_angle = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++]) //angle
    Line_position = Data_conversion(Identify_RX_2[cnt_I++], Identify_RX_2[cnt_I++])//position

}

//Voice recognition reception||语音识别接收
function voice_rx() {
    let rx_data = pins.createBuffer(4) //Create an array
    rx_data = serial.readBuffer(0) //Read data in RX buffer
    if (rx_data[0] == 0xF4 && rx_data[1] == 0x06 && rx_data[3] == 0xff) {
        get_data = rx_data[2]
    }
}

//Voice initialization||语音初始化
function Voice_init() {
    Quadruped.init()
    Quadruped.Height(10)
    Quadruped.Start()
}

//Voice data analysis||语音数据解析
function voice_data() {

    switch (get_data) {
        case 0x02: Voice_init(); break;
        case 0x03: Quadruped.Stop(); break;
        case 0x04: Quadruped.Reset; Quadruped.Stand(); break;
        case 0x05: Quadruped.Reset; Quadruped.Gait(gait.Trot); break;
        //case 0x06: Quadruped.Reset; Quadruped.Gait(gait.Crawl); break;
        case 0x07: Quadruped.Height(10); break;
        case 0x08: Quadruped.Height(4); break;
        case 0x09: Quadruped.Reset; Quadruped.Control_s(Mov_dir.For, voice_speed, 50); break;
        case 0x0A: Quadruped.Reset; Quadruped.Control_s(Mov_dir.Bac, voice_speed, 50); break;
        case 0x0B: Quadruped.Reset; Quadruped.Control_s(Mov_dir.Turn_l, voice_speed, 50); break;
        case 0x0C: Quadruped.Reset; Quadruped.Control_s(Mov_dir.Turn_r, voice_speed, 50); break;
        case 0x0D: Quadruped.Reset; Quadruped.Control_s(Mov_dir.Shift_l, voice_speed, 50); break;
        case 0x0E: Quadruped.Reset; Quadruped.Control_s(Mov_dir.Shift_r, voice_speed, 50); break;
        case 0x0F: Quadruped.Reset; Quadruped.Control_s(Mov_dir.For, voice_speed, 0); Quadruped.Control_s(Mov_dir.Shift_l, voice_speed, 50); break;
        case 0x10: Quadruped.Reset; Quadruped.Control_s(Mov_dir.Bac, voice_speed, 0); Quadruped.Control_s(Mov_dir.Shift_l, voice_speed, 50); break;
        case 0x11: Quadruped.Reset; Quadruped.Control_s(Mov_dir.For, voice_speed, 0); Quadruped.Control_s(Mov_dir.Shift_r, voice_speed, 50); break;
        case 0x12: Quadruped.Stand(); Quadruped.Control_s(Mov_dir.For, voice_speed, 0); Quadruped.Control_s(Mov_dir.Shift_l, voice_speed, 50); break;
        case 0x13: voice_speed++; break;
        case 0x14: voice_speed--; break;
        case 0x15: Quadruped.Reset; Quadruped.Control_a(Mov_ang.Look_d, 10, 20); break;
        case 0x16: Quadruped.Reset; Quadruped.Control_a(Mov_ang.Look_u, 10, 20); break;
        case 0x17: Quadruped.Reset; Quadruped.Control_a(Mov_ang.L_swing, 10, 20); break;
        case 0x18: Quadruped.Reset; Quadruped.Control_a(Mov_ang.R_swing, 10, 20); break;
        default: return

    }

}

//########gesture||手势
let Init_Register_Array = [
    [0xEF, 0x00],
    [0x37, 0x07],
    [0x38, 0x17],
    [0x39, 0x06],
    [0x41, 0x00],
    [0x42, 0x00],
    [0x46, 0x2D],
    [0x47, 0x0F],
    [0x48, 0x3C],
    [0x49, 0x00],
    [0x4A, 0x1E],
    [0x4C, 0x20],
    [0x51, 0x10],
    [0x5E, 0x10],
    [0x60, 0x27],
    [0x80, 0x42],
    [0x81, 0x44],
    [0x82, 0x04],
    [0x8B, 0x01],
    [0x90, 0x06],
    [0x95, 0x0A],
    [0x96, 0x0C],
    [0x97, 0x05],
    [0x9A, 0x14],
    [0x9C, 0x3F],
    [0xA5, 0x19],
    [0xCC, 0x19],
    [0xCD, 0x0B],
    [0xCE, 0x13],
    [0xCF, 0x64],
    [0xD0, 0x21],
    [0xEF, 0x01],
    [0x02, 0x0F],
    [0x03, 0x10],
    [0x04, 0x02],
    [0x25, 0x01],
    [0x27, 0x39],
    [0x28, 0x7F],
    [0x29, 0x08],
    [0x3E, 0xFF],
    [0x5E, 0x3D],
    [0x65, 0x96],
    [0x67, 0x97],
    [0x69, 0xCD],
    [0x6A, 0x01],
    [0x6D, 0x2C],
    [0x6E, 0x01],
    [0x72, 0x01],
    [0x73, 0x35],
    [0x74, 0x00],
    [0x77, 0x01]]

let Init_PS_Array = [
    [0xEF, 0x00],
    [0x41, 0x00],
    [0x42, 0x00],
    [0x48, 0x3C],
    [0x49, 0x00],
    [0x51, 0x13],
    [0x83, 0x20],
    [0x84, 0x20],
    [0x85, 0x00],
    [0x86, 0x10],
    [0x87, 0x00],
    [0x88, 0x05],
    [0x89, 0x18],
    [0x8A, 0x10],
    [0x9f, 0xf8],
    [0x69, 0x96],
    [0x6A, 0x02],
    [0xEF, 0x01],
    [0x01, 0x1E],
    [0x02, 0x0F],
    [0x03, 0x10],
    [0x04, 0x02],
    [0x41, 0x50],
    [0x43, 0x34],
    [0x65, 0xCE],
    [0x66, 0x0B],
    [0x67, 0xCE],
    [0x68, 0x0B],
    [0x69, 0xE9],
    [0x6A, 0x05],
    [0x6B, 0x50],
    [0x6C, 0xC3],
    [0x6D, 0x50],
    [0x6E, 0xC3],
    [0x74, 0x05]]

let Init_Gesture_Array = [
    [0xEF, 0x00],
    [0x41, 0x00],
    [0x42, 0x00],
    [0xEF, 0x00],
    [0x48, 0x3C],
    [0x49, 0x00],
    [0x51, 0x10],
    [0x83, 0x20],
    [0x9F, 0xF9],
    [0xEF, 0x01],
    [0x01, 0x1E],
    [0x02, 0x0F],
    [0x03, 0x10],
    [0x04, 0x02],
    [0x41, 0x40],
    [0x43, 0x30],
    [0x65, 0x96],
    [0x66, 0x00],
    [0x67, 0x97],
    [0x68, 0x01],
    [0x69, 0xCD],
    [0x6A, 0x01],
    [0x6B, 0xB0],
    [0x6C, 0x04],
    [0x6D, 0x2C],
    [0x6E, 0x01],
    [0x74, 0x00],
    [0xEF, 0x00],
    [0x41, 0xFF],
    [0x42, 0x01]]

const PAJ7620_ID = 0x73                   //Gesture recognition module address
const PAJ7620_REGITER_BANK_SEL = 0xEF     //Register bank selection

const PAJ7620_BANK0 = 0
const PAJ7620_BANK1 = 1

const GES_RIGHT_FLAG = 1
const GES_LEFT_FLAG = 2
const GES_UP_FLAG = 4
const GES_DOWN_FLAG = 8
const GES_FORWARD_FLAG = 16
const GES_BACKWARD_FLAG = 32
const GES_CLOCKWISE_FLAG = 64
const GES_COUNT_CLOCKWISE_FLAG = 128
const GES_WAVE_FLAG = 1

function GestureWriteReg(addr: number, cmd: number) {

    let buf = pins.createBuffer(2);
    buf[0] = addr;
    buf[1] = cmd;
    pins.i2cWriteBuffer(PAJ7620_ID, buf);
}

function GestureReadReg(addr: number): number {

    let buf = pins.createBuffer(1);
    buf[0] = addr;
    pins.i2cWriteBuffer(PAJ7620_ID, buf);

    let result = pins.i2cReadNumber(PAJ7620_ID, NumberFormat.UInt8LE, false);
    return result;
}




function GestureSelectBank(bank: number): void {
    switch (bank) {
        case 0:
            GestureWriteReg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK0);
            break;
        case 1:
            GestureWriteReg(PAJ7620_REGITER_BANK_SEL, PAJ7620_BANK1);
            break;
        default:
            break;
    }

}
