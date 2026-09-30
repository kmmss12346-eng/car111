/*
 * Fashion Star 总线伺服舵机驱动库
 * Version: v0.0.2
 * UpdateTime: 2024/07/17
 */
#include "fashion_star_uart_servo.h"
#include "math.h"

// 数据帧转换为字节数组
void FSUS_Package2RingBuffer(PackageTypeDef *pkg,  RingBufferTypeDef *ringBuf){
    uint8_t checksum; // 校验和
    // 写入帧头
    RingBuffer_WriteUShort(ringBuf, pkg->header);
    // 写入指令ID
    RingBuffer_WriteByte(ringBuf, pkg->cmdId);
    // 写入包的长度
    RingBuffer_WriteByte(ringBuf, pkg->size);
    // 写入内容主题
    RingBuffer_WriteByteArray(ringBuf, pkg->content, pkg->size);
    // 计算校验和
    checksum = RingBuffer_GetChecksum(ringBuf);
    // 写入校验和
    RingBuffer_WriteByte(ringBuf, checksum);

}

// 计算Package的校验和
uint8_t FSUS_CalcChecksum(PackageTypeDef *pkg){
    uint8_t checksum;
	// 初始化环形队列
	RingBufferTypeDef ringBuf;
	uint8_t pkgBuf[FSUS_PACK_RESPONSE_MAX_SIZE+1];
	RingBuffer_Init(&ringBuf, FSUS_PACK_RESPONSE_MAX_SIZE, pkgBuf);
    // 将Package转换为ringbuffer
	// 在转换的时候,会自动的计算checksum
    FSUS_Package2RingBuffer(pkg, &ringBuf);
	// 获取环形队列队尾的元素(即校验和的位置)
	checksum = RingBuffer_GetValueByIndex(&ringBuf, RingBuffer_GetByteUsed(&ringBuf)-1);
    return checksum;
}

// 判断是否为有效的请求头的
FSUS_STATUS FSUS_IsValidResponsePackage(PackageTypeDef *pkg){
    // 帧头数据不对
    if (pkg->header != FSUS_PACK_RESPONSE_HEADER){
        // 帧头不对
        return FSUS_STATUS_WRONG_RESPONSE_HEADER;
    }
    // 判断控制指令是否有效 指令范围超出
    if (pkg->cmdId > FSUS_CMD_NUM){
        return FSUS_STATUS_UNKOWN_CMD_ID;
    }
    // 参数的size大于FSUS_PACK_RESPONSE_MAX_SIZE里面的限制
    if (pkg->size > (FSUS_PACK_RESPONSE_MAX_SIZE - 5)){
        return FSUS_STATUS_SIZE_TOO_BIG;
    }
    // 校验和不匹配
    if (FSUS_CalcChecksum(pkg) != pkg->checksum){
        return FSUS_STATUS_CHECKSUM_ERROR;
    }
    // 数据有效
    return FSUS_STATUS_SUCCESS;
}

// 字节数组转换为数据帧
FSUS_STATUS FSUS_RingBuffer2Package(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg){
    // 申请内存
    pkg = (PackageTypeDef *)malloc(sizeof(PackageTypeDef));
    // 读取帧头
    pkg->header = RingBuffer_ReadUShort(ringBuf);
    // 读取指令ID
    pkg->cmdId = RingBuffer_ReadByte(ringBuf);
    // 读取包的长度
    pkg->size = RingBuffer_ReadByte(ringBuf);
    // 申请参数的内存空间
    // pkg->content = (uint8_t *)malloc(pkg->size);
    // 写入content
    RingBuffer_ReadByteArray(ringBuf, pkg->content, pkg->size);
    // 写入校验和
    pkg->checksum = RingBuffer_ReadByte(ringBuf);
    // 返回当前的数据帧是否为有效反馈数据帧
    return FSUS_IsValidResponsePackage(pkg);
}

// 构造发送数据帧
void FSUS_SendPackage(Usart_DataTypeDef *usart, uint8_t cmdId, uint8_t size, uint8_t *content){
    // 申请内存
	// printf("[Package] malloc for pkg\r\n");
    PackageTypeDef pkg;
	
    // 设置帧头
    pkg.header = FSUS_PACK_REQUEST_HEADER;
    // 设置指令ID
    pkg.cmdId = cmdId;
    // 设置尺寸
    pkg.size = size;
	// 逐一拷贝数组里面的内容
	for(int i=0; i<size; i++){
		pkg.content[i] = content[i];
	}
    // 将pkg发送到发送缓冲区sendBuf里面
    FSUS_Package2RingBuffer(&pkg, usart->sendBuf);
	// 通过串口将数据发送出去
    Usart_SendAll(usart);
}

// 接收数据帧 (在接收的时候动态的申请内存)
FSUS_STATUS FSUS_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg)
{
    uint8_t bIdx = 0;
    uint16_t header = 0;
    uint8_t rxByte;

    /* 记录开始时间 */
    uint32_t startTick = HAL_GetTick();

    /* 接收状态清零 */
    pkg->status = 0;

    /*
     * 最多等待 FSUS_TIMEOUT_MS 毫秒
     * HAL_GetTick() 会返回STM32启动以来经过的毫秒数
     */
    while ((HAL_GetTick() - startTick) < FSUS_TIMEOUT_MS)
    {
        /*
         * 如果Fashion驱动自己的接收缓冲区里没有数据，
         * 就直接从 USART6 读取一个字节。
         */
        if (RingBuffer_GetByteUsed(usart->recvBuf) == 0)
        {
            if (HAL_UART_Receive(&huart6,
                                 &rxByte,
                                 1,
                                 1) == HAL_OK)
            {
                /*
                 * USART6收到一个字节以后，
                 * 放进Fashion驱动的接收缓冲区。
                 */
                RingBuffer_Push(usart->recvBuf, rxByte);
            }
            else
            {
                continue;
            }
        }

        /*
         * 如果正文已经全部收到，
         * 那么接下来这个字节就是校验和。
         */
        if (pkg->status & FSUS_RECV_FLAG_CONTENT)
        {
            pkg->checksum =
                RingBuffer_ReadByte(usart->recvBuf);

            pkg->status |= FSUS_RECV_FLAG_CHECKSUM;

            /* 检查数据有没有传错 */
            if (FSUS_CalcChecksum(pkg) != pkg->checksum)
            {
                return FSUS_STATUS_CHECKSUM_ERROR;
            }

            return FSUS_STATUS_SUCCESS;
        }

        /*
         * 已经知道数据长度，
         * 现在开始一个字节一个字节接收正文。
         */
        else if (pkg->status & FSUS_RECV_FLAG_SIZE)
        {
            pkg->content[bIdx] =
                RingBuffer_ReadByte(usart->recvBuf);

            bIdx++;

            if (bIdx == pkg->size)
            {
                pkg->status |= FSUS_RECV_FLAG_CONTENT;
            }
        }

        /*
         * 命令ID已经收到，
         * 下一个字节表示数据长度。
         */
        else if (pkg->status & FSUS_RECV_FLAG_CMD_ID)
        {
            pkg->size =
                RingBuffer_ReadByte(usart->recvBuf);

            if (pkg->size >
                (FSUS_PACK_RESPONSE_MAX_SIZE - 5))
            {
                return FSUS_STATUS_SIZE_TOO_BIG;
            }

            pkg->status |= FSUS_RECV_FLAG_SIZE;
        }

        /*
         * 帧头已经收到，
         * 下一个字节是命令ID。
         */
        else if (pkg->status & FSUS_RECV_FLAG_HEADER)
        {
            pkg->cmdId =
                RingBuffer_ReadByte(usart->recvBuf);

            if (pkg->cmdId > FSUS_CMD_NUM)
            {
                return FSUS_STATUS_UNKOWN_CMD_ID;
            }

            pkg->status |= FSUS_RECV_FLAG_CMD_ID;
        }

        /*
         * 前面的内容都还没收到，
         * 说明现在正在寻找数据帧的帧头。
         */
        else
        {
            if (header == 0)
            {
                header =
                    RingBuffer_ReadByte(usart->recvBuf);

                if (header !=
                    (FSUS_PACK_RESPONSE_HEADER & 0x0F))
                {
                    header = 0;
                }
            }

            else if (header ==
                     (FSUS_PACK_RESPONSE_HEADER & 0x0F))
            {
                header =
                    header |
                    (RingBuffer_ReadByte(usart->recvBuf) << 8);

                if (header != FSUS_PACK_RESPONSE_HEADER)
                {
                    header = 0;
                }
                else
                {
                    pkg->header = header;

                    pkg->status |=
                        FSUS_RECV_FLAG_HEADER;
                }
            }
        }
    }

    /* 超过100ms还没有收到完整回复，就认为超时 */
    return FSUS_STATUS_TIMEOUT;
}

// 舵机通讯检测
// 注: 如果没有舵机响应这个Ping指令的话, 就会超时
FSUS_STATUS FSUS_Ping(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t statusCode; // 状态码
	uint8_t ehcoServoId; // PING得到的舵机ID
	// printf("[PING]Send Ping Package\r\n");
	// 发送请求包
	FSUS_SendPackage(usart, FSUS_CMD_PING, 1, &servo_id);
	// 接收返回的Ping
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);

    statusCode = FSUS_STATUS_SUCCESS;
	if(statusCode == FSUS_STATUS_SUCCESS){
		// 进一步检查ID号是否匹配
		ehcoServoId = (uint8_t)pkg.content[0];
		if (ehcoServoId != servo_id){
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}
	}
	return statusCode;
}

// 重置舵机的用户资料
FSUS_STATUS FSUS_ResetUserData(Usart_DataTypeDef *usart, uint8_t servo_id){
	const uint8_t size = 1;
	FSUS_STATUS statusCode;
	// 发送请求包
	FSUS_SendPackage(usart, FSUS_CMD_RESET_USER_DATA, size, &servo_id);
	// 接收重置结果
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 成功的接收到反馈数据
		// 读取反馈数据中的result
		uint8_t result = (uint8_t)pkg.content[1];
		if (result == 1){
			return FSUS_STATUS_SUCCESS;
		}else{
			return FSUS_STATUS_FAIL;
		}
	}
	return statusCode;
}

// 读取数据
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *usart, uint8_t servo_id,  uint8_t address, uint8_t *value, uint8_t *size){
	FSUS_STATUS statusCode;
	// 构造content
	uint8_t buffer[2] = {servo_id, address};
	// 发送请求数据
	FSUS_SendPackage(usart, FSUS_CMD_READ_DATA, 2, buffer);
	// 接收返回信息
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 读取数据
		// 读取数据是多少个位
		*size = pkg.size - 2; // content的长度减去servo_id跟address的长度
		// 数据拷贝
		for (int i=0; i<*size; i++){
			value[i] = pkg.content[i+2];
		}
	}
	return statusCode;
}

// 写入数据
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size){
	FSUS_STATUS statusCode;
	// 构造content
	uint8_t buffer[size+2]; // 舵机ID + 地址位Address + 数据byte数
	buffer[0] = servo_id;
	buffer[1] = address;
	// 拷贝数据
	for (int i=0; i<size; i++){
		buffer[i+2] = value[i];
	}
	// 发送请求数据
	FSUS_SendPackage(usart, FSUS_CMD_WRITE_DATA, size+2, buffer);
	// 接收返回信息
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		uint8_t result = pkg.content[2];
		if(result == 1){
			statusCode = FSUS_STATUS_SUCCESS;
		}else{
			statusCode = FSUS_STATUS_FAIL;
		}
	}
	return statusCode;
}



/* 
 * 轮式控制模式
 * speed单位 °/s
 */
FSUS_STATUS FSUS_WheelMove(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t method, uint16_t speed, uint16_t value){
	// 创建环形缓冲队列
	const uint8_t size = 6;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	// 写入content
	RingBuffer_WriteByte(&ringBuf, servo_id);  // 舵机ID
	RingBuffer_WriteByte(&ringBuf, method);   // 写入执行方式与旋转方向
	RingBuffer_WriteUShort(&ringBuf, speed);  // 设置舵机的旋转速度 °/s
	RingBuffer_WriteUShort(&ringBuf, value);
	// 发送请求包
	FSUS_SendPackage(usart, FSUS_CMD_SPIN, size, buffer+1);
	
	return FSUS_STATUS_SUCCESS;
}

// 轮式模式, 舵机停止转动
FSUS_STATUS FSUS_WheelStop(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t method = 0x00;
	uint16_t speed = 0;
	uint16_t value = 0;
	return FSUS_WheelMove(usart, servo_id, method, speed, value);
}

// 轮式模式 不停旋转
FSUS_STATUS FSUS_WheelKeepMove(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t is_cw, uint16_t speed){
	uint8_t method = 0x01; // 持续旋转
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	uint16_t value = 0;
	return FSUS_WheelMove(usart, servo_id, method, speed, value);
}

// 轮式模式 按照特定的速度旋转特定的时间
FSUS_STATUS FSUS_WheelMoveTime(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t is_cw, uint16_t speed, uint16_t nTime){
	uint8_t method = 0x03; // 旋转一段时间
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	return FSUS_WheelMove(usart, servo_id, method, speed, nTime);
}

// 轮式模式 旋转特定的圈数
FSUS_STATUS FSUS_WheelMoveNCircle(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t is_cw, uint16_t speed, uint16_t nCircle){
	uint8_t method = 0x02; // 旋转特定的圈数
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	return FSUS_WheelMove(usart, servo_id, method, speed, nCircle);
}

// 设置舵机的角度
// @angle 单位度
// @interval 单位ms
// @power 舵机执行功率 单位mW
//        若power=0或者大于保护值
FSUS_STATUS FSUS_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 7;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10*angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, power);
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_ROTATE, size, buffer+1);
	
	if (wait){
		return FSUS_Wait(usart, servo_id, angle, 0); 
	}else{
		return FSUS_STATUS_SUCCESS;
	}
}

/* 设置舵机的角度(指定周期) */
FSUS_STATUS FSUS_SetServoAngleByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float angle, uint16_t interval, uint16_t t_acc, \
				uint16_t t_dec, uint16_t  power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	if (t_acc < 20){
		t_acc = 20;
	}
	if (t_dec < 20){
		t_dec = 20;
	}
	
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10*angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 0);  
	}else{
		return FSUS_STATUS_SUCCESS;
	}
	
}

/* 设置舵机的角度(指定转速) */
FSUS_STATUS FSUS_SetServoAngleByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float angle, float velocity, uint16_t t_acc, \
				uint16_t t_dec, uint16_t  power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	if(velocity < 1.0f){
		velocity = 1.0f;
	}else if(velocity > 750.0f){
		velocity = 750.0f;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10.0f*angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 0); 
	}else{
		return FSUS_STATUS_SUCCESS;
	}
	
}

/* 查询单个舵机的角度信息 angle 单位度 */
FSUS_STATUS FSUS_QueryServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
	const uint8_t size = 1; // 请求包content的长度
	uint8_t ehcoServoId;
	int16_t echoAngle;
	
	// 发送舵机角度请求包
	FSUS_SendPackage(usart, FSUS_CMD_READ_ANGLE, size, &servo_id);
	// 接收返回的Ping
	PackageTypeDef pkg;
	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 成功的获取到舵机角度回读数据
		ehcoServoId = (uint8_t)pkg.content[0];
		// 检测舵机ID是否匹配
		if (ehcoServoId != servo_id){
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}
		
		// 提取舵机角度
		echoAngle = (int16_t)(pkg.content[1] | (pkg.content[2] << 8));
		*angle = (float)(echoAngle / 10.0);
	}
  return statusCode;
}	

/* 设置舵机的角度(多圈模式) */
FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
	uint32_t interval, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束			
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(interval > 4096000){
		interval = 4096000;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10*angle));
	RingBuffer_WriteULong(&ringBuf, interval);
	RingBuffer_WriteShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 1);
	}else{
		return FSUS_STATUS_SUCCESS;
	}
}

/* 设置舵机的角度(多圈模式, 指定周期) */
FSUS_STATUS FSUS_SetServoAngleMTurnByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
			uint32_t interval,  uint16_t t_acc,  uint16_t t_dec, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 15;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	
	// 数值约束			
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(interval > 4096000){
		interval = 4096000;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10*angle));
	RingBuffer_WriteULong(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 1);
	}else{
		return FSUS_STATUS_SUCCESS;
	}
}

/* 设置舵机的角度(多圈模式, 指定转速) */
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
			float velocity, uint16_t t_acc,  uint16_t t_dec, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 13;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(velocity < 1.0f){
		velocity = 1.0f;
	}else if(velocity > 750.0f){
		velocity = 750.0f;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10.0f*angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 1);
	}else{
		return FSUS_STATUS_SUCCESS;
	}
	
}

/* 查询舵机的角度(多圈模式) */
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
	// 创建环形缓冲队列
	const uint8_t size = 1; // 请求包content的长度
	uint8_t ehcoServoId;
	int32_t echoAngle;
	
	// 发送舵机角度请求包
	FSUS_SendPackage(usart, FSUS_CMD_QUERY_SERVO_ANGLE_MTURN, size, &servo_id);
	// 接收返回的Ping
	PackageTypeDef pkg;
	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 成功的获取到舵机角度回读数据
		ehcoServoId = (uint8_t)pkg.content[0];
		// 检测舵机ID是否匹配
		if (ehcoServoId != servo_id){
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}
		
		// 提取舵机角度
		echoAngle = (int32_t)(pkg.content[1] | (pkg.content[2] << 8) |  (pkg.content[3] << 16) | (pkg.content[4] << 24));
		*angle = (float)(echoAngle / 10.0);
	}
  return statusCode;
}


/* 舵机阻尼模式 */
FSUS_STATUS FSUS_DampingMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint16_t power){
	const uint8_t size = 3; // 请求包content的长度
	uint8_t buffer[size+1]; // content缓冲区
	RingBufferTypeDef ringBuf; // 创建环形缓冲队列
	RingBuffer_Init(&ringBuf, size, buffer); // 缓冲队列初始化
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteUShort(&ringBuf, power);
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_DAMPING, size, buffer+1);
	return FSUS_STATUS_SUCCESS;
}


/* 等待电机旋转到特定的位置 */
FSUS_STATUS FSUS_Wait(Usart_DataTypeDef *usart, uint8_t servo_id, float target_angle, uint8_t is_mturn){
	float angle_read; // 读取出来的电机角度
	uint16_t count = 0; // 计数
	float angle_error;
	// 循环等待
	while(1){
		if(is_mturn){
			 FSUS_QueryServoAngleMTurn(usart, servo_id, &angle_read);
		}else{
			 FSUS_QueryServoAngle(usart, servo_id, &angle_read);
		}
		
		angle_error = fabsf(target_angle - angle_read);
		// printf("status:%d, angle:%.1f ,angle_error: %.1f\r\n", status, angle_read, angle_error);
		if(angle_error <= FSUS_ANGLE_DEADAREA){
			return FSUS_STATUS_SUCCESS;
		}
		
		// 超时判断机制
		count += 1;
		if(count>=FSUS_WAIT_COUNT_MAX){
			return FSUS_STATUS_FAIL;
		}
		// 延时100ms
		HAL_Delay(100);
	}
	
}

// 舵机重置多圈角度圈数
FSUS_STATUS FSUS_ServoAngleReset(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t statusCode; // 状态码

	FSUS_SendPackage(usart, FSUS_CMD_RESERT_SERVO_ANGLE_MTURN, 1, &servo_id);
	// 接收返回的Ping

	return statusCode;
}

/*零点设置 仅适用于无刷磁编码舵机*/
FSUS_STATUS FSUS_SetOriginPoint(Usart_DataTypeDef *usart, uint8_t servo_id)
{
	uint8_t statusCode; // 状态码
	uint8_t data[2]={0};
	data[1]=servo_id;
	FSUS_SendPackage(usart, FSUS_CMD_SET_ORIGIN_POINT, 2, data);
	// 接收返回的Ping

	return statusCode;
}
