/**
 * Quadruped
 */
//% weight= 0 color=#0abcff icon="\uf201" block="Quadruped"
//% groups='["Set up","control","Additional steering gear control","Joint angle control"]'
namespace Quadruped {

    /**
     *TODO: Define the communication pins and related settings of the microbit and the adapter board (initialization is required before basic control, external servo control, and joint control). After initialization, the LED dot matrix screen of the microbit will not work
     *TODO:定义microbit和转接板的通讯引脚及相关设置（基本控制、外接舵机控制、关节控制前都需进行初始化）。初始化后microbit的LED点阵屏将无法使用
     */
    //% group="Set up"
    //% blockGap=8
    //% blockId=Quadruped_init block="init"
    export function init(): void {
        SPI_Init()
    }
    //###return hexadecimal number||返回状态信息
    /**
    * TODO:Returns the status information of the robot itself (0x00 idle, 0x01 powered on, 0x04, trot 0x06 crawling, 0x7 recovering, 0x08 fell)
    * TODO:返回机器人自身的状态信息（0x00空闲，0x01上电，0x04，小跑0x06爬行，0x7恢复中，0x08摔倒）
    */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Status block="Status"
    export function Status(): number {
        return robot_mode;
    }
    //###return hexadecimal number||Returns action status information
    /**
    * TODO:Returns the status information of the robot itself (0x00 idle, 0x01 powered on, 0x04, trot 0x06 crawling, 0x7 recovering, 0x08 fell)
    * TODO:返回机器人自身的状态信息（0x00空闲，0x01上电，0x04，小跑0x06爬行，0x7恢复中，0x08摔倒）
    */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Action_Status block="Action_Status"
    export function Action_Status(): number {
        return robot_mode;
    }	
	
    //####Reset||复位
    /**
     *TODO:Movement speed and attitude angle reset to 0
     *TODO:移动速度和姿态角重置为0
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Reset block="Reset"
    export function Reset(): void {
        rc_spd_cmd_X = 0.00 //x_speed
        rc_spd_cmd_y = 0.00 //y_speed
        rc_att_rate_cmd = 0.00 // Turn to speed
        rc_spd_cmd_z = 0.00 //Altitude speed
        //rc_pos_cmd = 0.00 //height
        rc_att_cmd_x = 0.00 //Pitch
        rc_att_cmd_y = 0.00 //Side swing
        rc_att_cmd = 0.00 //Heading
    }
    //####Height||高度
    /**
     * TODO: Robot body height adjustment (0-10 range adjustment, 0 is the lowest, 10 is the highest)
     * TODO: 机器人本体高度调节（0-10范围调节，0最低，10最高）
     */
    //% group="control"
    //% blockGap=8
    //% h.min=0.00 h.max=10.00
    //% blockId=Quadruped_Height block="Height %h"
    export function Height(h: number): void {
        rc_pos_cmd = h * 0.1
        for (let i = 0; i < 10; i++)
        {
            SPI_Send()
            basic.pause(100)
        }
        
    }
    //###Start||启动
    /**
     * TODO:The robot is powered on and enters the half-squat state (internally starts to send commands, the basic control needs to be initialized and started before other building blocks can be used)
     * TODO:机器人上电进入半蹲状态（内部开始发送指令，基本控制需要先初始化启动才能使用其他积木）
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Start block="Start"
    export function Start(): void {
        gait_mode = 4
        state = 1
        basic.pause(3000)
        //serial.writeNumber(3)
        while (1) {
            SPI_Send()
            if (robot_mode == 1) {
                for (let i = 0; i < 5; i++) {
                    SPI_Send()
                    basic.pause(100)
                }
                return
            }
            //serial.writeNumber(4)
        }
    }
    //###Quadruped Stand||站立
    /**
     * TODO:The machine enters the standing mode (it needs to stop in place when trotting and crawling, you can add this block to enter the standing mode)
     * TODO:机器进入站立模式（小跑和爬行时需要原地停止，可以加这个积木进入站立模式）
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Stand block="Stand"
    export function Stand(): void {
        Standing()
    }
    //####Quadruped Fall recovery||摔倒恢复
    /**
     * TODO:Automatically detect whether to enter fall mode, then automatically enter fall recovery
     * TODO:自动检测是否进入跌倒模式，然后自动进入跌倒恢复
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Fall_recovery block="Fall recovery"
    export function Fall_re(): void {
        if (robot_mode != 0x08)
            return
        if (robot_mode == 0x08) {
            gait_mode = 0x07
            SPI_Send()
            robot_mode_1 = robot_mode
            while (robot_mode_1 != 0x07) {
                return
            }
        }
    }
    //###Heartbeat||心跳
    /**
     * TODO:This block needs to be placed in a loop (to prevent loss of communication with the machine)
     * TODO:此方块需要放在循环中（防止与机器通讯丢失）
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Heartbeat block="Heartbeat"
    export function Heartbeat(): void {
        SPI_Send()
        //serial.writeNumber(10)
    }
    //###Stop||停止
    /**
     * TODO:Enter the shutdown mode, the fuselage squat (internally stop sending commands)
     * TODO:进入关机模式，机身下蹲（内部停止发送指令）
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Stop block="Stop"
    export function Stop(): void {
        if (robot_mode == 0x04||robot_mode == 0x06) {
            Standing()
        }
        if (robot_mode == 1 || robot_mode == 0X02) {
            rc_pos_cmd = 0.01
        }
        SPI_Send()
        basic.pause(50)
        SPI_Send()
        state = 0
    }
    //###gait||步态
    /**
     * TODO:Two options: trot and crawling (note: crawling gait can only be used when the fuselage is at its highest)
     * TODO:两种选择：小跑和爬行（注意：爬行步态只能在机身处于最高状态时使用）
     */
    //% group="control"
    //% blockGap=8
    //% blockId=Quadruped_Gait block="Gait | %g"
    export function Gait(g: gait): void {
        switch (g) {
            case gait.Trot:
                gait_mode = 0x01;
                while (1) {
                    SPI_Send()
                    if (robot_mode == 0x04) {
                        SPI_Send()
                        //serial.writeNumber(2)
                        return
                    }
                }
            case gait.Crawl:
                rc_pos_cmd = 1
                for (let i = 0; i < 5; i++)
                {
                    SPI_Send()
                    basic.pause(100)
                    }
                gait_mode = 0x03;
                while (1) {
                    SPI_Send()
                    if (robot_mode == 0x06) {
                        SPI_Send()
                        //serial.writeNumber(2)
                        return
                   }
                }
            case gait.S_TROT:
                rc_pos_cmd = 1
                for (let i = 0; i < 5; i++)
                {
                    SPI_Send()
                    basic.pause(100)
                    }
                gait_mode = 0x0D;
                while (1) {
                    SPI_Send()
                    if (robot_mode == 0x0D) {
                        SPI_Send()
                        return
                   }
                }
            case gait.F_TROT:
                rc_pos_cmd = 1
                for (let i = 0; i < 5; i++)
                {
                    SPI_Send()
                    basic.pause(100)
                    }
                gait_mode = 0x01;
                while (1) {
                    SPI_Send()
                    if (robot_mode == 0x04) {
                        SPI_Send()
                        //serial.writeNumber(2)
                        break
                    }
                }			
                gait_mode = 0x02;
                while (1) {
                    SPI_Send()
                    if (robot_mode == 0x05) {
                        SPI_Send()
                        return
                   }
                }			
        }
        SPI_Send()
    }
	
	
    //###Action group||动作组
    /**
    * TODO:Back and forth, left and right movement, left and right rotation speed control. time in seconds
    * TODO:前后、左右移动、左右旋转速度控制。时间以秒为单位
    */
    //% group="control"
    //% blockGap=8
    //% Group.min=0 Group.max=10
    //% time1.min=0 time1.max=255
    //% blockId=Quadruped_Action_groups block="Action group|%Group|state %sta"
    export function Action_groups(Group: number, sta: Actions): void {
	  Action_group =   Group
	  if(sta == Actions.Enable){
		Action_group_status = 1
	  }
	  else{
	  	Action_group_status = 1
	  }
	  SPI_Send()
		  
		  
    }
    //###Movement direction and speed||运动方向与速度
    /**
    * TODO:Back and forth, left and right movement, left and right rotation speed control. time in seconds
    * TODO:前后、左右移动、左右旋转速度控制。时间以秒为单位
    */
    //% group="control"
    //% blockGap=8
    //% speed1.min=0.00 speed1.max=10.00
    //% time1.min=0 time1.max=255
    //% blockId=Quadruped_Control_s block="Control direction| %m|speed %speed1|time %time1"
    export function Control_s(m: Mov_dir, speed1: number, time1: number): void {
        let Sum_S = 0.00
        let time_ms = 0
        let time_s = time1*1000
        let time_start = 0
        Sum_S = speed1 / 100.00
        SPI_Send()
        switch (m) {
            case Mov_dir.For:
                rc_spd_cmd_X = Sum_S; SPI_Send(); break;
            case Mov_dir.Bac:
                rc_spd_cmd_X = (-Sum_S); SPI_Send(); break;
            case Mov_dir.Turn_l:
                rc_att_rate_cmd = (speed1 * 5); SPI_Send(); break;
            case Mov_dir.Turn_r:
                rc_att_rate_cmd = (-speed1 * 5); SPI_Send(); break;
            case Mov_dir.Shift_l:
                rc_spd_cmd_y = (-Sum_S); SPI_Send(); break;
            case Mov_dir.Shift_r:
                rc_spd_cmd_y = Sum_S; SPI_Send(); break;
        }
        //for (let e = 0; e < time1; e++) {
        //    SPI_Send()
        //    basic.pause(1000)
        //}
        time_start = input.runningTime()
        while(1){
            time_ms = input.runningTime() - time_start
            SPI_Send()
            if(time_s <= time_ms)
                return
        }
    }
    //###Control angle||控制角度
    /**
    * TODO:Angle control for left and right swing, head up, head down and left and right twist. time in seconds
    * TODO:左右摆动、抬头、低头和左右扭转的角度控制。时间以秒为单位
    */
    //% group="control"
    //% blockGap=8
    //% angle1.min=0.00 angle1.max=10.00
    //% time1.min=0 time1.max=255
    //% blockId=Quadruped_Control_a block="Control angle |%m|angle_size %angle1|time %time1"
    export function Control_a(m: Mov_ang, angle1: number, time1: number): void {
        let time_ms = 0
        let time_s = time1*1000
        let time_start = 0
        switch (m) {
            case Mov_ang.Look_d:
                rc_att_cmd_x = angle1; break;
            case Mov_ang.Look_u:
                rc_att_cmd_x = (-angle1); break;
            case Mov_ang.L_swing:
                if (angle1 == 0) {
                    rc_att_cmd_y = 0; break;
                }
                else {
                    rc_att_cmd_y = angle1 + 10; break;
                }
            case Mov_ang.R_swing:
                if (angle1 == 0) {
                    rc_att_cmd_y = 0; break;
                }
                else {
                    rc_att_cmd_y = (-angle1) - 10; break;
                }
            case Mov_ang.Yaw_l:
                rc_att_cmd = angle1; break;
            case Mov_ang.Yaw_r:
                rc_att_cmd = -(angle1); break;
        }
        //for (let e = 0; e < time1; e++) {
        //    SPI_Send()
        //    basic.pause(1000)
        //}
        time_start = input.runningTime()
        while(1){
            time_ms = input.runningTime() - time_start
            SPI_Send()
            if(time_s <= time_ms)
                return
        }
        
    }

    //###Joint angle control||关节控制
    /**
    * TODO:Select the corresponding leg, joint angle and whether to execute the command of the current building block
    * TODO:选择对应的腿、关节角度以及是否执行当前积木的命令
    */
    //% group="Joint angle control"
    //% blockGap=9
    //% blockId=Quadruped_Joint block="Joint angle control | %j|thigh %d|Calf %x|Side led %c| %site "
    export function Joint(j: Joints, d: number, x: number, c: number, site: sIte): void {
        switch (j) {
            case Joints.Left_fr: FL_d = d; FL_x = x; FL_c = c; break;
            case Joints.Left_hi: HL_d = d; HL_x = x; HL_c = c; break
            case Joints.Right_fr: FR_d = d; FR_x = x; FR_c = c; break
            case Joints.Right_hi: HR_d = d; HR_x = x; HR_c = c; break
        }
        if (site = 1)
            Joint_SPI_Send()
    }

    //###Joint Heartbeat||关节心跳
    /**
    * TODO:Continuously send the command information set in the previous step (to prevent the loss of machine communication)
    * TODO:不断发送上一步设置的命令信息（防止机器通讯丢失）
    */
    //% group="Joint angle control"
    //% blockGap=8
    //% blockId=Joint_Heartbeat block="Joint Heartbeat"
    export function Joint_Heartbeat(): void {
            Joint_SPI_Send()
    }


    //###Ultrasound||超声波
    /**
    * TODO:Select the transmit and receive pins corresponding to the ultrasonic wave, and select the unit of the returned data
    * TODO:选择超声波对应的发射和接收引脚，并选择返回数据的单位
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Model block="Ultrasound |tr %trig |re %echo | unit %unit"
    export function Ultrasound(trig: DigitalPin, echo: DigitalPin, unit: Unit, maxCmDistance = 500): number {
        // send pulse
        maxCmDistance = 500
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);
        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);
        switch (unit) {
            case Unit.Centimeters: return Math.idiv(d, 58);
            case Unit.Inches: return Math.idiv(d, 148);
            default: return d;
        }
    }
    //###Infrared||红外
    /**
     * TODO:Select data receiving pin (0 obstacle, 1 no recognition)
     * TODO:选择数据接收引脚（0障碍物，1无识别到）
     */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Infrared block="Infrared |mode %value |pin %pin"
    export function Infrared(pin: DigitalPin): number {
        pins.setPull(pin, PinPullMode.PullUp);
        return pins.digitalReadPin(pin);
    }
    //###Human body induction||人体感应
    /**
    * TODO:Select the data receiving pin (1 recognizes the human body, 0 does not recognize)
    * TODO:选择数据接收引脚（1识别到人体，0无识别到）
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Human_Infrared block="Human Infrared|pin|%pin"
    export function Human_induction(pin: AnalogPin, value=50): number {
        let w = pins.analogReadPin(pin)
        if (w >= value)
            return 1
        return 0
    }
    //###GestureInit||手势初始化
    /**
    * IODO:Gesture related pins, configuration settings (success: 0 fail: 255)
    * IODO:手势相关引脚、配置设置（成功：0 失败：255）
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_GestureInit block="GestureInit"
    export function GestureInit(): number {
        basic.pause(800);//等待芯片稳定
        if (GestureReadReg(0) != 0x20) {
            return 0xff;
        }
        for (let i = 0; i < Init_Register_Array.length; i++) {
            GestureWriteReg(Init_Register_Array[i][0], Init_Register_Array[i][1]);
        }
        GestureSelectBank(0);
        for (let i = 0; i < Init_Gesture_Array.length; i++) {
            GestureWriteReg(Init_Gesture_Array[i][0], Init_Gesture_Array[i][1]);
        }
        return 0;
    }
    //###GetGesture||获取手势
    /**
    * IODO:Returns the value of the gesture direction
    * IODO:返回手势方向的值
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_GetGesture block="GetGesture"
    export function GetGesture(): number {

        let date = GestureReadReg(0x43);

        switch (date) {
            case GES_RIGHT_FLAG:
            case GES_LEFT_FLAG:
            case GES_UP_FLAG:
            case GES_DOWN_FLAG:
            case GES_FORWARD_FLAG:
            case GES_BACKWARD_FLAG:
            case GES_CLOCKWISE_FLAG:
            case GES_COUNT_CLOCKWISE_FLAG:
                break;
            default:
                date = GestureReadReg(0x44);
                if (date == GES_WAVE_FLAG) {
                    return 256;
                }
                break;
        }
        return date;
    }

    //###Select_gesture_as||选择手势为
    /**
    * IODO:A value that defines the direction of the gesture
    * IODO:定义手势方向的值
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Select_gesture_as block="Select_gesture_as | %state"
    export function Select_gesture_as(state: gesture): number {
        return state;
    }

    //###Steering gear control||舵机控制
    /**
    * IODO:Select external control pin, PWM value range: 500-2500 (0°~180°), speed: servo response speed: 1~10 (1 fast~10 slow)
    * IODO:选择外部控制引脚，PWM值范围：500-2500（0°~180°），速度：舵机响应速度：1~10（1快~10慢）
    */
    //% group="Additional steering gear control"
    //% blockGap=8
    //% h.min=0 h.max=3
    //% pwm.min=500 pwm.max=2500
    //% Gap.min=1 Gap.max=9
    //% blockId=sensor_Steering_gear block="Steering_gear| %h | PWM_value %pwm|Rotation speed %Gap"
    export function Steering_gear(h: number, pwm: number, Gap: number) {
        usb_send_cnt_1 = 0;

        ToSlaveBuf_1[usb_send_cnt_1++] = DaHeader_1; //head
        ToSlaveBuf_1[usb_send_cnt_1++] = SSLen - 2; //Fixed length
        ToSlaveBuf_1[usb_send_cnt_1++] = 2;  //function code

        ToSlaveBuf_1[usb_send_cnt_1++] = h;
        ToSlaveBuf_1[usb_send_cnt_1++] = pwm >> 8;
        ToSlaveBuf_1[usb_send_cnt_1++] = (pwm << 8) >> 8;
        ToSlaveBuf_1[usb_send_cnt_1++] = Gap;

        ToSlaveBuf_1[SSLen - 1] = DaTail_1;//Fixed length

        SG_SPI_Send()
    }


  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //###Image recognition initialization||图像识别初始化
    /**
    * IODO:Image recognition internal related pins and settings initialization
    * IODO:图像识别内部相关引脚和设置初始化
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Image_ini block="Image recognition initialization"
    export function Image_init() {
        serial.setRxBufferSize(32)
    }

	//OnToggleIdentify 开启/切换(颜色、标签、二维码)
    /**
     * IODO:Set pattern recognition function (color, label)
     * IODO:设置图形识别功能（颜色、标签）
     */
     //% subcategory=sensor
     //% blockGap=8
     //% blockId=OnToggle block="OnToggle|%Fun"
    export function OnToggle(Fun:  FunctionID): void { 
        for (let i = 1; i < 2; i++)
            {
            IRecognitionToggle()    //复位
        }
        FrameHeader = 0xAA
        DataID = 0x03
        FunID = Fun
        IRecognitionSettings()
    }

	//OnToggle1 开启/切换(小球、形状、巡线)
    /**
     * IODO:Set the graphic recognition function (ball, shape, line) and the corresponding recognition color
     * IODO:设置图形识别功能（小球、形状、线）以及对应识别颜色
     */
     //% subcategory=sensor
     //% blockGap=8
     //% blockId=OnToggle1 block="OnToggle1| %Col|%Fun"
     export function OnToggle1(Col: ColorID,Fun:  FunctionID1): void { 
        for (let i = 1; i < 2; i++)
        {
            IRecognitionToggle()    //复位
        }
        FrameHeader = 0xAA
        DataID = 0x01
        ColID = Col
        FunID = Fun
        IRecognitionSettings()
    } 
    
 	//OnToggle2 开启/切换(巡线)
    /**
     * IODO:Set the graphic recognition function (line) and the corresponding recognition color
     * IODO:设置图形识别功能(线）以及对应识别颜色
     */
     //% subcategory=sensor
     //% blockGap=8
     //% blockId=OnToggle2 block="OnToggle2| %Col|Line"
     export function OnToggle2(Col: ColorLineID): void { 
        for (let i = 1; i < 2; i++)
        {
            IRecognitionToggle()    //复位
        }
        FrameHeader = 0xAA
        DataID = 0x01
        ColID = Col
        FunID = 0x04
        IRecognitionSettings()
    }    
    
	//TogetherOn 开启/切换巡线+形状同是识别
    /**
     * IODO:At the same time, open the setting pattern recognition function line patrol + line patrol and the corresponding recognition color
     * IODO:同时开启设置图形识别功能巡线+巡线以及对应识别颜色
     */
     //% subcategory=sensor
     //% blockGap=8
     //% blockId=TogetherOn block="TogetherOn| %Col|Line|%Col2|Shape"
     export function TogetherOn(Col1: ColorLineID,Col2: ColorID): void { 
        for (let i = 1; i < 2; i++)
        {
            IRecognitionToggle()    //复位
        }
        FrameHeader = 0xAA
        DataID = 0x04
        ColID = Col1
        ShaColID = Col2
        FunID = 0x08 
        IRecognitionSettings()
    }       

    //###Tag code position return value||标签位置返回值
    /**
    * IODO: Returns the value of the Tag code set, flips the X, y, Z, XYZ axes (corresponding to the position and flip angle)
    * IODO:返回Tag码集的值，翻转X、y、Z、XYZ轴（对应位置和翻转角度）
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Tag_return block="Tag code position return value| %data"
    export function Tag_return(data: Position): number {
        Function_c = 0x19
        Function_s = 2
        Identify_send()
        Identify_receive()
        switch (data) {
            case Position.Content_V: return Identify_pattern; break;
            case Position.X_axis: return Identify_x; break;
            case Position.Y_axis: return Identify_x; break;
            case Position.Z_axis: return Identify_x; break;
            case Position.X_flip: return Identify_x; break;
            case Position.Y_flip: return Identify_x; break;
            case Position.Z_flip: return Identify_x; break;
            default: return 255
        }
    }
    


    //###Ball return value||小球返回值
    /**
    * IODO:Returns the ball information, the recognition status (returns 1 and 0, 1 means recognized, 0 means unrecognized), the X and Y axis positions of the center of the ball in the image, the two-dimensional width and height of the ball, and the recognition effect (the higher the recognition effect, the smaller the distance of the ball). return value (float)
    * IODO:返回小球信息，识别状态（返回1和0，1表示已识别，0表示未识别），图像中小球中心的X、Y轴位置，二维宽高 小球，以及识别效果（识别效果越高，小球的距离）。返回值（浮点数）
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Ball_return block="Ball returnvalue| %P"
    export function Ball_return(P: Ball_Position): number {
        Function_c = 0x0F
        Function_s = 3
        Identify_send()
        Identify_receive()
        switch (P) {
            case Ball_Position.status: return Ball_status
            case Ball_Position.X_axis: return Ball_X
            case Ball_Position.Y_axis: return Ball_Y
            case Ball_Position.Width: return Ball_W
            case Ball_Position.Depth: return Ball_H
            case Ball_Position.Re_effect: return Ball_pixels
            default: return 255
        }

    }

    //###Line patrol return||巡线返回
    /**
    * IODO:Status (return 1 and 0, 1 is recognized, 0 is not recognized), recognition effect (the pixel value of the recognition line is 0-19200), deviation angle (-90°--0°--90°), deviation X-axis position (- 160 - 0 - 160,) return value (float)
    * IODO:状态（返回1和0，1识别，0未识别到），识别效果（识别线像素值大小为0-19200），偏差角度（-90°--0°--90°），偏差 X 轴位置 (- 160 - 0 - 160,) 返回值 (float)
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=sensor_Line_return block="Line patrol return value| %x"
    export function Line_return(X: Line_Position): number {
        Function_c = 0x23
        Function_s = 4
        Identify_send()
        Identify_receive()
        switch (X) {
            case Line_Position.status: return Line_detect;
            case Line_Position.Re_effect: return Line_effect;
            case Line_Position.De_angle: return Line_angle;
            case Line_Position.De_position: return Line_position;
            default: return 255
        }
    }

    //###ColorRecognitionreturn||颜色返回
    /**
    * IODO:Color returns 16bit number 1 (red) 2 (blue) 4 (yellow) 8 (green) 3 (with red and black) 5 (red and yellow) 6 (yellow and blue) 7 (red, blue and yellow) 9 (green and red)
    * IODO:颜色返回16bit数字1（红色）2（蓝色）4（黄色）8（绿色）3（有红色和黑色）5（红黄）6（黄蓝）7（红蓝黄）9（绿红）
    * 
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=ColorRecognition block="Color Recognition return value"
    export function Colorreturn(): number {
        Function_c = 0x41
        Function_s = 1
        Identify_send()
        Identify_receive()
        return Color_ID;
    }

    //###ShapeRecognitionreturn||形状返回
    /**
    * IODO:Recognized shape returns 0~3 numbers (0: none, 1: triangle, 2: rectangle, 3: circle)
    * IODO:识别形状返回0~3数字（0：无、1：三角形、2：矩形、3：圆形）
    */
    //% subcategory=sensor
    //% blockGap=8
    //% blockId=ShapeRecognition block="shape recognition returns"
    export function Shapereturn(): number {
        Function_c = 0x23
        Function_s = 5
        Identify_send()
        Identify_receive()
        return Shapes_ID;
    }



}
