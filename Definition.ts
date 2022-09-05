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
let Action_group = 0x00
let Action_group_status = 0x00
let M_Action_group_status = 0x00

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

/* ************************************基本控制******************************** */

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
    ToSlaveBuf[usb_send_cnt++] = Action_group_status
    ToSlaveBuf[usb_send_cnt++] = Action_group
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
    if (InfoTemp[0] == 0x2B && InfoTemp[2] == 0x80) {
        robot_mode = InfoTemp[3]
        M_Action_group_status = InfoTemp[4]
    }
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

//#################################Data conversion||数据转换######################################################
function DecToBinTail(dec: number, pad: number) {
    let bin = "";
    let i;
    for (i = 0; i < pad; i++) {
        dec *= 2;
        if (dec >= 1) {
            dec -= 1;
            bin += "1";
        }
        else {
            bin += "0";
        }
    }
    return bin;
}

function DecToBinHead(dec: number, pad: number) {
    let bin = "";
    let i;
    for (i = 0; i < pad; i++) {
        bin = parseInt((dec % 2).toString()) + bin;
        dec /= 2;
    }
    return bin;
}

function get_float_hex(decString: number) {
    let dec = decString;
    let sign;
    let signString;
    let decValue = parseFloat(Math.abs(decString).toString());
    let fraction = 0;
    let exponent = 0;
    let ssss = []

    if (decString.toString().charAt(0) == '-') {
        sign = 1;
        signString = "1";
    }
    else {
        sign = 0;
        signString = "0";
    }
    if (decValue == 0) {
        fraction = 0;
        exponent = 0;
    }
    else {
        exponent = 127;
        if (decValue >= 2) {
            while (decValue >= 2) {
                exponent++;
                decValue /= 2;
            }
        }
        else if (decValue < 1) {
            while (decValue < 1) {
                exponent--;
                decValue *= 2;
                if (exponent == 0)
                    break;
            }
        }
        if (exponent != 0) decValue -= 1; else decValue /= 2;

    }
    let fractionString = DecToBinTail(decValue, 23);
    let exponentString = DecToBinHead(exponent, 8);
    let ss11 = parseInt(signString + exponentString + fractionString, 2)
    ToSlaveBuf[usb_send_cnt++] = ((ss11 << 24) >> 24)
    ToSlaveBuf[usb_send_cnt++] = ((ss11 << 16) >> 24)
    ToSlaveBuf[usb_send_cnt++] = ((ss11 << 8) >> 24)
    ToSlaveBuf[usb_send_cnt++] = ((ss11 >> 24))
}

/* ************************************传感器******************************** */
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

/* ************************************图像识别******************************** */
let num = 0
let Header = 0xaa //包头DaHeader
let Tail = 0xAB //包尾DaTail
let Color_id = 0 //基础颜色位置
let Linecolor_id = 0 //线颜色位置
let Find_ID = 0 //接收功能ID
let Function_ID = 0 //发送功能ID
let Identity_RX = pins.createBuffer(22)  //接收的缓冲区 Identify_RX
let Select_Tx = pins.createBuffer(5)  //开启功能选择：包头，功能，基础颜色位置，线颜色，包尾 //


//mall ball  //bal->ball
let Ball_status = 0x00  //status
let Ball_X = 0x00, Ball_Y = 0x00 //x-axis, y-axis
let Ball_W = 0x00, Ball_H = 0x00 //Width Height
let Ball_pixels = 0x00  //Number of pixels

//Line inspection  //lin->line
let Line_detect = 0x00 //Detect
let Line_effect = 0x00 //The effect of the identification line
let Line_angle = 0x00 //angle
let Line_position = 0x00 //position

let Shape_ID = 0 //Shapes_ID

//QR code
let Identity_x = 0x00, Identity_y = 0x00, Identity_z = 0x00
let Identity_Flip_x = 0x00, Identity_Flip_y = 0x00, Identity_Flip_z = 0x00
let Identity_status = 0x00, Identity_pattern = 0x00


//功能和颜色选择发送
function Uart_sent() { //IRecognitionSetting
    num = 0
    Select_Tx[0] = Header
    Select_Tx[1] = Function_ID //功能
    Select_Tx[2] = Color_id //基础颜色
    Select_Tx[3] = Linecolor_id //线颜色
    Select_Tx[4] = Tail
    serial.writeBuffer(Select_Tx)
    basic.pause(100)
}


// 功能切换     数组清空赋0
function Reset() { //IRecognitionToggle
    num = 0
    Select_Tx[0] = Header
    Select_Tx[1] = 0x00
    Select_Tx[2] = 0x00
    Select_Tx[3] = 0x00
    Select_Tx[4] = Tail
    serial.writeBuffer(Select_Tx)
    //basic.pause(100)
}

//串口接收
function Uart_receive() {
    Identity_RX = serial.readBuffer(0)//若要避免等待数据，请将长度设置为立即返回缓冲数据:0
    if (Identity_RX[0] == Header && Identity_RX[21] == Tail) {
        Color_id = Identity_RX[1]

        Ball_status = Identity_RX[2]
        Ball_X = Identity_RX[3]
        Ball_Y = Identity_RX[4]
        Ball_W = Identity_RX[5]
        Ball_H = Identity_RX[6]
        Ball_pixels = Identity_RX[7]

        Line_detect = Identity_RX[8]
        Line_effect = Identity_RX[9]
        Line_angle = Identity_RX[10]
        Line_position = Identity_RX[11]

        Shape_ID = Identity_RX[12]

        Identity_status = Identity_RX[13]
        Identity_pattern = Identity_RX[14]
        Identity_x = Identity_RX[15]
        Identity_y = Identity_RX[16]
        Identity_z = Identity_RX[17]
        Identity_Flip_x = Identity_RX[18]
        Identity_Flip_y = Identity_RX[19]
        Identity_Flip_z = Identity_RX[20]

    }
}


