import threading
import time
import queue
import random
import struct
import math

class FloatCPUSimulator:
    def __init__(self, memory_size=256, stack_size=16):
        # 8位寄存器
        self.A = 0  # 累加器
        self.B = 0  # 通用寄存器
        self.C = 0  # 通用寄存器
        self.D = 0  # 通用寄存器
        self.PC = 0  # 程序计数器
        self.SP = 0  # 栈指针 (指向栈顶)
        self.IX = 0  # 变址寄存器
        self.FLAGS = 0  # 标志寄存器 (Z:零标志, C:进位标志, S:符号标志, I:中断允许标志)
        
        # 浮点寄存器 (32位单精度)
        self.F0 = 0.0  # 浮点寄存器0
        self.F1 = 0.0  # 浮点寄存器1
        self.F2 = 0.0  # 浮点寄存器2
        self.F3 = 0.0  # 浮点寄存器3
        
        # 内存 (256字节)
        self.memory = [0] * memory_size
        
        # 栈 (16字节)
        self.stack = [0] * stack_size
        self.stack_size = stack_size
        
        # 浮点栈 (用于浮点运算)
        self.fp_stack = [0.0] * 8
        self.fp_sp = 0  # 浮点栈指针
        
        # 中断相关
        self.interrupt_queue = queue.PriorityQueue()  # 优先级中断队列
        self.current_priority = -1  # 当前处理的中断优先级 (-1表示无中断)
        self.interrupt_mask = 0xFF   # 中断掩码 (默认允许所有中断)
        self.interrupt_enabled = True  # 全局中断允许标志
        self.nesting_level = 0  # 中断嵌套层级
        
        # 中断优先级定义 (0-7, 0为最高优先级)
        self.interrupt_priorities = {
            0: 0,  # 最高优先级中断 (如复位)
            1: 2,  # 高优先级中断 (如硬件故障)
            2: 4,  # 中优先级中断 (如定时器)
            3: 6,  # 低优先级中断 (如I/O)
        }
        
        # 内存映射I/O地址定义
        self.io_map = {
            0xF0: "TIMER_CTRL",    # 定时器控制寄存器
            0xF1: "TIMER_VALUE",   # 定时器当前值
            0xF2: "UART_DATA",     # UART数据寄存器
            0xF3: "UART_STATUS",   # UART状态寄存器
            0xF4: "GPIO_DATA",     # GPIO数据寄存器
            0xF5: "GPIO_DIR",      # GPIO方向寄存器
            0xF6: "ADC_VALUE",     # ADC值寄存器
            0xF7: "PWM_DUTY",      # PWM占空比寄存器
            0xF8: "INTERRUPT_CTRL",# 中断控制寄存器
            0xF9: "INTERRUPT_MASK",# 中断掩码寄存器
            0xFA: "SYSTEM_CTRL",   # 系统控制寄存器
            0xFB: "RTC_SECONDS",   # RTC秒寄存器
            0xFC: "RTC_MINUTES",   # RTC分钟寄存器
            0xFD: "RTC_HOURS",     # RTC小时寄存器
            0xFE: "DEVICE_ID",     # 设备ID寄存器
            0xFF: "SYSTEM_STATUS", # 系统状态寄存器
        }
        
        # I/O设备状态
        self.io_devices = {
            "TIMER_CTRL": 0,      # 定时器控制: bit0-启用, bit1-自动重载
            "TIMER_VALUE": 0,     # 定时器当前值
            "UART_DATA": 0,       # UART数据
            "UART_STATUS": 0,     # UART状态: bit0-数据就绪, bit1-发送就绪
            "GPIO_DATA": 0,       # GPIO数据
            "GPIO_DIR": 0,        # GPIO方向: 0-输入, 1-输出
            "ADC_VALUE": 0,       # ADC值 (模拟输入)
            "PWM_DUTY": 0,        # PWM占空比
            "INTERRUPT_CTRL": 0,  # 中断控制: bit0-全局中断使能
            "INTERRUPT_MASK": 0xFF, # 中断掩码
            "SYSTEM_CTRL": 0,     # 系统控制
            "RTC_SECONDS": 0,     # RTC秒
            "RTC_MINUTES": 0,     # RTC分钟
            "RTC_HOURS": 0,       # RTC小时
            "DEVICE_ID": 0x42,    # 设备ID
            "SYSTEM_STATUS": 0,   # 系统状态
        }
        
        # 模拟设备状态
        self.timer_counter = 0
        self.uart_input_buffer = []
        self.gpio_input_state = 0
        self.adc_value = 0
        self.rtc_seconds = 0
        self.rtc_minutes = 0
        self.rtc_hours = 0
        
        # 指令集映射
        self.instructions = {
            # 基本指令
            0x00: self.hlt,   # 停止执行
            0x01: self.lda,   # 加载立即数到A
            0x02: self.ldb,   # 加载立即数到B
            0x03: self.ldc,   # 加载立即数到C
            0x04: self.ldd,   # 加载立即数到D
            0x05: self.sta,   # 存储A到内存
            0x06: self.stb,   # 存储B到内存
            0x07: self.stc,   # 存储C到内存
            0x08: self.std,   # 存储D到内存
            0x09: self.add,   # A = A + B
            0x0A: self.sub,   # A = A - B
            0x0B: self.mul,   # A = A * B
            0x0C: self.div,   # A = A / B
            0x0D: self.inc,   # A = A + 1
            0x0E: self.dec,   # A = A - 1
            0x0F: self.jmp,   # 跳转到地址
            0x10: self.jz,    # 如果零标志为真则跳转
            0x11: self.jnz,   # 如果零标志为假则跳转
            0x12: self.jc,    # 如果进位标志为真则跳转
            0x13: self.jnc,   # 如果进位标志为假则跳转
            0x14: self.js,    # 如果符号标志为真则跳转
            0x15: self.jns,   # 如果符号标志为假则跳转
            0x16: self.out,   # 输出A的值
            0x17: self.nop,   # 空操作
            
            # 逻辑运算指令
            0x18: self.and_,  # A = A & B
            0x19: self.or_,   # A = A | B
            0x1A: self.xor,   # A = A ^ B
            0x1B: self.not_,  # A = ~A
            
            # 移位指令
            0x1C: self.shl,   # 算术左移
            0x1D: self.shr,   # 算术右移
            0x1E: self.rol,   # 循环左移
            0x1F: self.ror,   # 循环右移
            
            # 栈操作指令
            0x20: self.push,  # 将A压入栈
            0x21: self.pop,   # 从栈弹出到A
            0x22: self.call,  # 调用子程序
            0x23: self.ret,   # 从子程序返回
            
            # 数据传输指令
            0x24: self.mov_ab, # MOV A, B
            0x25: self.mov_ac, # MOV A, C
            0x26: self.mov_ad, # MOV A, D
            0x27: self.mov_ba, # MOV B, A
            0x28: self.mov_bc, # MOV B, C
            0x29: self.mov_bd, # MOV B, D
            0x2A: self.mov_ca, # MOV C, A
            0x2B: self.mov_cb, # MOV C, B
            0x2C: self.mov_cd, # MOV C, D
            0x2D: self.mov_da, # MOV D, A
            0x2E: self.mov_db, # MOV D, B
            0x2F: self.mov_dc, # MOV D, C
            
            # 比较指令
            0x30: self.cmp,   # 比较A和B，设置标志位
            
            # 输入指令
            0x31: self.in_,    # 从输入读取到A
            
            # 中断指令
            0x32: self.ei,     # 允许中断
            0x33: self.di,     # 禁止中断
            0x34: self.iret,   # 从中断返回
            0x35: self.int,    # 软件中断
            
            # I/O指令
            0x36: self.in_io,  # 从I/O端口读取到A
            0x37: self.out_io, # 从A输出到I/O端口
            
            # 变址寄存器指令
            0x38: self.ldx,    # 加载立即数到IX
            0x39: self.stx,    # 存储IX到内存
            0x3A: self.inx,    # IX递增
            0x3B: self.dcx,    # IX递减
            
            # 寻址模式指令
            0x40: self.lda_direct,     # LDA 直接寻址
            0x41: self.lda_indirect,   # LDA 间接寻址
            0x42: self.lda_indexed,    # LDA 变址寻址
            0x43: self.sta_direct,     # STA 直接寻址
            0x44: self.sta_indirect,   # STA 间接寻址
            0x45: self.sta_indexed,    # STA 变址寻址
            0x46: self.add_direct,     # ADD 直接寻址
            0x47: self.add_indirect,   # ADD 间接寻址
            0x48: self.add_indexed,    # ADD 变址寻址
            0x49: self.jmp_indirect,   # JMP 间接寻址
            0x4A: self.jmp_indexed,    # JMP 变址寻址
            
            # 浮点指令
            0x50: self.fld,       # 加载浮点数到F0
            0x51: self.fst,       # 存储F0到内存
            0x52: self.fadd,      # 浮点加法: F0 = F0 + F1
            0x53: self.fsub,      # 浮点减法: F0 = F0 - F1
            0x54: self.fmul,      # 浮点乘法: F0 = F0 * F1
            0x55: self.fdiv,      # 浮点除法: F0 = F0 / F1
            0x56: self.fsqrt,     # 浮点平方根: F0 = sqrt(F0)
            0x57: self.fsin,      # 浮点正弦: F0 = sin(F0)
            0x58: self.fcos,      # 浮点余弦: F0 = cos(F0)
            0x59: self.ftan,      # 浮点正切: F0 = tan(F0)
            0x5A: self.fcmp,      # 浮点比较: 比较F0和F1
            0x5B: self.fmov,      # 浮点移动: F0 = F1
            0x5C: self.fpush,     # 浮点压栈
            0x5D: self.fpop,      # 浮点出栈
            0x5E: self.fint,      # 浮点转整数: A = (int)F0
            0x5F: self.ffloat,    # 整数转浮点: F0 = (float)A
            0x60: self.fout,      # 输出浮点数F0
        }
        
        # 初始化中断向量表 (地址 0x00-0x0F)
        # 每个中断向量占2字节 (低地址字节是处理程序地址低8位，高地址字节是高8位)
        self.memory[0x00] = 0x20  # 中断0处理程序地址 (0x0020)
        self.memory[0x01] = 0x00
        self.memory[0x02] = 0x30  # 中断1处理程序地址 (0x0030)
        self.memory[0x03] = 0x00
        self.memory[0x04] = 0x40  # 中断2处理程序地址 (0x0040)
        self.memory[0x05] = 0x00
        self.memory[0x06] = 0x50  # 中断3处理程序地址 (0x0050)
        self.memory[0x07] = 0x00
        
        # 初始化中断处理程序
        self.setup_interrupt_handlers()
        
        # 启动设备模拟线程
        self.running = True
        self.device_thread = threading.Thread(target=self.device_simulator)
        self.device_thread.daemon = True
        self.device_thread.start()
    
    def setup_interrupt_handlers(self):
        """设置中断处理程序"""
        # 中断0处理程序: 最高优先级中断 (模拟系统复位)
        self.memory[0x20] = 0x01  # LDA
        self.memory[0x21] = 0xFF  # 立即数 0xFF
        self.memory[0x22] = 0x05  # STA
        self.memory[0x23] = 0xF0  # 存储到地址 0xF0 (系统状态)
        self.memory[0x24] = 0x34  # IRET
        
        # 中断1处理程序: 高优先级中断 (模拟硬件故障)
        self.memory[0x30] = 0x01  # LDA
        self.memory[0x31] = 0xEE  # 立即数 0xEE
        self.memory[0x32] = 0x05  # STA
        self.memory[0x33] = 0xF1  # 存储到地址 0xF1 (错误代码)
        self.memory[0x34] = 0x34  # IRET
        
        # 中断2处理程序: 中优先级中断 (模拟定时器中断)
        self.memory[0x40] = 0x36  # IN_IO
        self.memory[0x41] = 0xF1  # 从TIMER_VALUE读取
        self.memory[0x42] = 0x05  # STA
        self.memory[0x43] = 0x80  # 存储到地址 0x80 (定时器值备份)
        self.memory[0x44] = 0x34  # IRET
        
        # 中断3处理程序: 低优先级中断 (模拟UART中断)
        self.memory[0x50] = 0x36  # IN_IO
        self.memory[0x51] = 0xF2  # 从UART_DATA读取
        self.memory[0x52] = 0x05  # STA
        self.memory[0x53] = 0x81  # 存储到地址 0x81 (接收数据)
        self.memory[0x54] = 0x34  # IRET
    
    def device_simulator(self):
        """设备模拟线程"""
        while self.running:
            # 模拟定时器
            if self.io_devices["TIMER_CTRL"] & 0x01:  # 定时器启用
                self.timer_counter += 1
                if self.timer_counter >= 10:  # 每10次计数触发一次中断
                    self.timer_counter = 0
                    self.io_devices["TIMER_VALUE"] = (self.io_devices["TIMER_VALUE"] + 1) & 0xFF
                    # 触发定时器中断
                    self.trigger_interrupt(2)
            
            # 模拟UART输入
            if random.random() < 0.1:  # 10%的概率有输入
                if not self.uart_input_buffer:
                    # 生成随机输入
                    char = random.randint(65, 90)  # A-Z
                    self.uart_input_buffer.append(char)
                
                if self.uart_input_buffer and not (self.io_devices["UART_STATUS"] & 0x01):
                    self.io_devices["UART_DATA"] = self.uart_input_buffer.pop(0)
                    self.io_devices["UART_STATUS"] |= 0x01  # 设置数据就绪标志
                    # 触发UART中断
                    self.trigger_interrupt(3)
            
            # 模拟ADC输入
            self.adc_value = (self.adc_value + random.randint(-5, 5)) & 0xFF
            self.io_devices["ADC_VALUE"] = self.adc_value
            
            # 模拟RTC
            self.rtc_seconds = (self.rtc_seconds + 1) % 60
            if self.rtc_seconds == 0:
                self.rtc_minutes = (self.rtc_minutes + 1) % 60
                if self.rtc_minutes == 0:
                    self.rtc_hours = (self.rtc_hours + 1) % 24
            
            self.io_devices["RTC_SECONDS"] = self.rtc_seconds
            self.io_devices["RTC_MINUTES"] = self.rtc_minutes
            self.io_devices["RTC_HOURS"] = self.rtc_hours
            
            # 模拟GPIO输入
            self.gpio_input_state = (self.gpio_input_state + random.randint(0, 1)) & 0x0F
            # 只有配置为输入的位才反映外部状态
            input_mask = ~self.io_devices["GPIO_DIR"] & 0xFF
            self.io_devices["GPIO_DATA"] = (self.io_devices["GPIO_DATA"] & self.io_devices["GPIO_DIR"]) | (self.gpio_input_state & input_mask)
            
            time.sleep(0.1)  # 每100ms更新一次设备状态
    
    def read_io(self, address):
        """从I/O设备读取数据"""
        if address in self.io_map:
            device_name = self.io_map[address]
            
            # 特殊处理某些设备
            if device_name == "UART_DATA":
                # 读取UART数据后清除数据就绪标志
                self.io_devices["UART_STATUS"] &= ~0x01
            
            return self.io_devices[device_name]
        else:
            print(f"警告: 读取未映射的I/O地址 0x{address:02X}")
            return 0
    
    def write_io(self, address, value):
        """向I/O设备写入数据"""
        if address in self.io_map:
            device_name = self.io_map[address]
            
            # 特殊处理某些设备
            if device_name == "UART_DATA":
                # 写入UART数据
                print(f"UART输出: {chr(value)} (0x{value:02X})")
                # 设置发送就绪标志
                self.io_devices["UART_STATUS"] |= 0x02
            
            elif device_name == "GPIO_DATA":
                # 只有配置为输出的位才能被写入
                output_mask = self.io_devices["GPIO_DIR"]
                self.io_devices[device_name] = (value & output_mask) | (self.io_devices[device_name] & ~output_mask)
                print(f"GPIO输出: 0x{self.io_devices[device_name]:02X}")
            
            elif device_name == "INTERRUPT_CTRL":
                # 中断控制寄存器
                self.interrupt_enabled = bool(value & 0x01)
                self.io_devices[device_name] = value & 0x01
            
            elif device_name == "INTERRUPT_MASK":
                # 中断掩码寄存器
                self.interrupt_mask = value
                self.io_devices[device_name] = value
            
            else:
                # 普通写入
                self.io_devices[device_name] = value & 0xFF
        else:
            print(f"警告: 写入未映射的I/O地址 0x{address:02X}")
    
    def reset(self):
        """重置CPU状态"""
        self.A = 0
        self.B = 0
        self.C = 0
        self.D = 0
        self.PC = 0
        self.SP = 0
        self.IX = 0
        self.FLAGS = 0
        self.F0 = 0.0
        self.F1 = 0.0
        self.F2 = 0.0
        self.F3 = 0.0
        self.memory = [0] * len(self.memory)
        self.stack = [0] * self.stack_size
        self.fp_stack = [0.0] * 8
        self.fp_sp = 0
        
        # 清空中断队列
        while not self.interrupt_queue.empty():
            try:
                self.interrupt_queue.get_nowait()
            except queue.Empty:
                break
        
        self.current_priority = -1
        self.interrupt_mask = 0xFF
        self.interrupt_enabled = True
        self.nesting_level = 0
        
        # 重置I/O设备
        for device in self.io_devices:
            self.io_devices[device] = 0
        self.io_devices["DEVICE_ID"] = 0x42
        
        # 重置模拟设备状态
        self.timer_counter = 0
        self.uart_input_buffer = []
        self.gpio_input_state = 0
        self.adc_value = 0
        self.rtc_seconds = 0
        self.rtc_minutes = 0
        self.rtc_hours = 0
        
        # 重新初始化中断向量表和处理程序
        self.setup_interrupt_handlers()
    
    def load_program(self, program, start_address=0):
        """将程序加载到内存中"""
        for i, byte in enumerate(program):
            if start_address + i < len(self.memory):
                self.memory[start_address + i] = byte
    
    def fetch_byte(self):
        """从内存中获取一个字节并递增PC"""
        byte = self.memory[self.PC]
        self.PC = (self.PC + 1) % len(self.memory)
        return byte
    
    def check_interrupt(self):
        """检查是否有中断请求，考虑优先级"""
        if not self.interrupt_enabled or self.interrupt_queue.empty():
            return
        
        # 获取最高优先级的中断
        highest_priority = None
        highest_vector = None
        
        # 由于PriorityQueue不能遍历，我们需要临时存储和重新添加
        temp_queue = queue.PriorityQueue()
        
        while not self.interrupt_queue.empty():
            priority, vector = self.interrupt_queue.get()
            temp_queue.put((priority, vector))
            
            # 检查中断是否被屏蔽
            if not (self.interrupt_mask & (1 << vector)):
                continue
                
            # 检查优先级是否高于当前处理的中断
            if priority < self.current_priority or self.current_priority == -1:
                if highest_priority is None or priority < highest_priority:
                    highest_priority = priority
                    highest_vector = vector
        
        # 将中断重新放回队列
        while not temp_queue.empty():
            priority, vector = temp_queue.get()
            if priority != highest_priority or vector != highest_vector:
                self.interrupt_queue.put((priority, vector))
        
        # 处理最高优先级中断
        if highest_vector is not None:
            self.handle_interrupt(highest_vector, highest_priority)
    
    def handle_interrupt(self, vector, priority):
        """处理中断"""
        print(f"处理中断 {vector} (优先级: {priority}), 当前优先级: {self.current_priority}")
        
        # 保存当前程序状态
        # 压入PC、FLAGS和当前优先级到栈中
        self.push(self.PC & 0xFF)        # PC低8位
        self.push((self.PC >> 8) & 0xFF) # PC高8位
        self.push(self.FLAGS)            # FLAGS
        self.push(self.current_priority) # 当前优先级
        
        # 设置新的当前优先级
        self.current_priority = priority
        self.nesting_level += 1
        
        # 获取中断处理程序地址
        vector_addr = vector * 2  # 每个向量占2字节
        handler_low = self.memory[vector_addr]
        handler_high = self.memory[vector_addr + 1]
        handler_addr = (handler_high << 8) | handler_low
        
        # 跳转到中断处理程序
        self.PC = handler_addr
        
        print(f"跳转到中断处理程序 0x{handler_addr:04X}, 嵌套层级: {self.nesting_level}")
    
    def trigger_interrupt(self, vector):
        """触发中断"""
        if vector < 0 or vector > 3:
            print(f"无效的中断向量: {vector}")
            return
        
        priority = self.interrupt_priorities.get(vector, 7)  # 默认优先级7（最低）
        self.interrupt_queue.put((priority, vector))
        print(f"中断请求: 向量 {vector}, 优先级 {priority}")
    
    def execute(self, max_cycles=1000):
        """执行程序"""
        running = True
        cycles = 0
        
        while running and cycles < max_cycles:
            # 检查中断
            self.check_interrupt()
            
            # 获取并执行指令
            opcode = self.fetch_byte()
            if opcode in self.instructions:
                running = self.instructions[opcode]()
            else:
                print(f"未知操作码: 0x{opcode:02X}")
                running = False
            cycles += 1
            
        if cycles >= max_cycles:
            print(f"达到最大循环次数 ({max_cycles})，停止执行")
        else:
            print("程序执行结束")
        
        self.running = False
    
    def update_flags(self, value):
        """根据值更新标志寄存器"""
        self.FLAGS &= 0xF9  # 清除零标志和符号标志，保留进位标志和中断允许标志
        
        # 设置零标志
        if value == 0:
            self.FLAGS |= 0x02  # 设置零标志
        
        # 设置符号标志 (最高位)
        if value & 0x80:
            self.FLAGS |= 0x04  # 设置符号标志
    
    def float_to_bytes(self, f):
        """将浮点数转换为4字节"""
        return struct.pack('f', f)
    
    def bytes_to_float(self, b):
        """将4字节转换为浮点数"""
        return struct.unpack('f', b)[0]
    
    # 指令实现
    def hlt(self):
        """停止执行"""
        return False
    
    def lda(self):
        """加载立即数到A (立即寻址)"""
        self.A = self.fetch_byte()
        self.update_flags(self.A)
        return True
    
    def ldb(self):
        """加载立即数到B (立即寻址)"""
        self.B = self.fetch_byte()
        return True
    
    def ldc(self):
        """加载立即数到C (立即寻址)"""
        self.C = self.fetch_byte()
        return True
    
    def ldd(self):
        """加载立即数到D (立即寻址)"""
        self.D = self.fetch_byte()
        return True
    
    def sta(self):
        """存储A到内存 (直接寻址)"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            self.write_io(addr, self.A)
        else:
            self.memory[addr] = self.A
        return True
    
    def stb(self):
        """存储B到内存 (直接寻址)"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            self.write_io(addr, self.B)
        else:
            self.memory[addr] = self.B
        return True
    
    def stc(self):
        """存储C到内存 (直接寻址)"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            self.write_io(addr, self.C)
        else:
            self.memory[addr] = self.C
        return True
    
    def std(self):
        """存储D到内存 (直接寻址)"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            self.write_io(addr, self.D)
        else:
            self.memory[addr] = self.D
        return True
    
    def add(self):
        """A = A + B (寄存器寻址)"""
        result = self.A + self.B
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置进位标志
        if result > 0xFF:
            self.FLAGS |= 0x01  # 设置进位标志
            result &= 0xFF  # 截断到8位
        
        self.A = result
        self.update_flags(self.A)
        return True
    
    def sub(self):
        """A = A - B (寄存器寻址)"""
        result = self.A - self.B
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置进位标志(借位)
        if result < 0:
            self.FLAGS |= 0x01  # 设置进位标志
            result &= 0xFF  # 截断到8位
        
        self.A = result
        self.update_flags(self.A)
        return True
    
    def mul(self):
        """A = A * B (寄存器寻址)"""
        result = self.A * self.B
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置进位标志(如果结果超过8位)
        if result > 0xFF:
            self.FLAGS |= 0x01  # 设置进位标志
            result &= 0xFF  # 截断到8位
        
        self.A = result
        self.update_flags(self.A)
        return True
    
    def div(self):
        """A = A / B (寄存器寻址)"""
        if self.B == 0:
            print("错误: 除以零")
            return False
        
        result = self.A // self.B
        self.A = result
        self.update_flags(self.A)
        return True
    
    def inc(self):
        """A = A + 1 (隐含寻址)"""
        self.A = (self.A + 1) & 0xFF
        self.update_flags(self.A)
        return True
    
    def dec(self):
        """A = A - 1 (隐含寻址)"""
        self.A = (self.A - 1) & 0xFF
        self.update_flags(self.A)
        return True
    
    def jmp(self):
        """跳转到地址 (直接寻址)"""
        self.PC = self.fetch_byte()
        return True
    
    def jz(self):
        """如果零标志为真则跳转 (直接寻址)"""
        addr = self.fetch_byte()
        if self.FLAGS & 0x02:  # 检查零标志
            self.PC = addr
        return True
    
    def jnz(self):
        """如果零标志为假则跳转 (直接寻址)"""
        addr = self.fetch_byte()
        if not (self.FLAGS & 0x02):  # 检查零标志
            self.PC = addr
        return True
    
    def jc(self):
        """如果进位标志为真则跳转 (直接寻址)"""
        addr = self.fetch_byte()
        if self.FLAGS & 0x01:  # 检查进位标志
            self.PC = addr
        return True
    
    def jnc(self):
        """如果进位标志为假则跳转 (直接寻址)"""
        addr = self.fetch_byte()
        if not (self.FLAGS & 0x01):  # 检查进位标志
            self.PC = addr
        return True
    
    def js(self):
        """如果符号标志为真则跳转 (直接寻址)"""
        addr = self.fetch_byte()
        if self.FLAGS & 0x04:  # 检查符号标志
            self.PC = addr
        return True
    
    def jns(self):
        """如果符号标志为假则跳转 (直接寻址)"""
        addr = self.fetch_byte()
        if not (self.FLAGS & 0x04):  # 检查符号标志
            self.PC = addr
        return True
    
    def out(self):
        """输出A的值 (隐含寻址)"""
        print(f"输出: {self.A} (0x{self.A:02X})")
        return True
    
    def nop(self):
        """空操作 (隐含寻址)"""
        return True
    
    def and_(self):
        """A = A & B (寄存器寻址)"""
        self.A &= self.B
        self.update_flags(self.A)
        return True
    
    def or_(self):
        """A = A | B (寄存器寻址)"""
        self.A |= self.B
        self.update_flags(self.A)
        return True
    
    def xor(self):
        """A = A ^ B (寄存器寻址)"""
        self.A ^= self.B
        self.update_flags(self.A)
        return True
    
    def not_(self):
        """A = ~A (隐含寻址)"""
        self.A = (~self.A) & 0xFF
        self.update_flags(self.A)
        return True
    
    def shl(self):
        """算术左移: A << 1, 最高位进入进位标志 (隐含寻址)"""
        carry = (self.A & 0x80) >> 7  # 获取最高位
        self.A = (self.A << 1) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def shr(self):
        """算术右移: A >> 1, 最低位进入进位标志 (隐含寻址)"""
        carry = self.A & 0x01  # 获取最低位
        self.A = (self.A >> 1) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def rol(self):
        """循环左移: 进位标志参与循环 (隐含寻址)"""
        carry = (self.A & 0x80) >> 7  # 获取最高位
        self.A = ((self.A << 1) | (self.FLAGS & 0x01)) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def ror(self):
        """循环右移: 进位标志参与循环 (隐含寻址)"""
        carry = self.A & 0x01  # 获取最低位
        self.A = ((self.A >> 1) | ((self.FLAGS & 0x01) << 7)) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def push(self, value=None):
        """将值压入栈 (隐含寻址)"""
        if value is None:
            value = self.A
        
        if self.SP < self.stack_size:
            self.stack[self.SP] = value
            self.SP += 1
        else:
            print("栈溢出")
        return True
    
    def pop(self):
        """从栈弹出到A (隐含寻址)"""
        if self.SP > 0:
            self.SP -= 1
            self.A = self.stack[self.SP]
            self.update_flags(self.A)
        else:
            print("栈下溢")
        return True
    
    def call(self):
        """调用子程序: 将返回地址压栈并跳转 (直接寻址)"""
        return_addr = self.PC + 1  # 跳过地址字节
        if self.SP < self.stack_size - 1:
            # 压入返回地址
            self.push(return_addr & 0xFF)        # 低8位
            self.push((return_addr >> 8) & 0xFF) # 高8位
            
            # 跳转到目标地址
            self.PC = self.fetch_byte()
        else:
            print("栈溢出")
        return True
    
    def ret(self):
        """从子程序返回: 从栈中弹出返回地址 (隐含寻址)"""
        if self.SP > 1:
            # 弹出返回地址高8位
            self.SP -= 1
            high_byte = self.stack[self.SP]
            
            # 弹出返回地址低8位
            self.SP -= 1
            low_byte = self.stack[self.SP]
            
            self.PC = (high_byte << 8) | low_byte
        else:
            print("栈下溢")
        return True
    
    # 数据传输指令
    def mov_ab(self):
        """MOV A, B (寄存器寻址)"""
        self.A = self.B
        self.update_flags(self.A)
        return True
    
    def mov_ac(self):
        """MOV A, C (寄存器寻址)"""
        self.A = self.C
        self.update_flags(self.A)
        return True
    
    def mov_ad(self):
        """MOV A, D (寄存器寻址)"""
        self.A = self.D
        self.update_flags(self.A)
        return True
    
    def mov_ba(self):
        """MOV B, A (寄存器寻址)"""
        self.B = self.A
        return True
    
    def mov_bc(self):
        """MOV B, C (寄存器寻址)"""
        self.B = self.C
        return True
    
    def mov_bd(self):
        """MOV B, D (寄存器寻址)"""
        self.B = self.D
        return True
    
    def mov_ca(self):
        """MOV C, A (寄存器寻址)"""
        self.C = self.A
        return True
    
    def mov_cb(self):
        """MOV C, B (寄存器寻址)"""
        self.C = self.B
        return True
    
    def mov_cd(self):
        """MOV C, D (寄存器寻址)"""
        self.C = self.D
        return True
    
    def mov_da(self):
        """MOV D, A (寄存器寻址)"""
        self.D = self.A
        return True
    
    def mov_db(self):
        """MOV D, B (寄存器寻址)"""
        self.D = self.B
        return True
    
    def mov_dc(self):
        """MOV D, C (寄存器寻址)"""
        self.D = self.C
        return True
    
    def cmp(self):
        """比较A和B，设置标志位 (寄存器寻址)"""
        result = self.A - self.B
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置零标志
        if result == 0:
            self.FLAGS |= 0x02
        else:
            self.FLAGS &= 0xFD  # 清除零标志
        
        # 设置进位标志(借位)
        if result < 0:
            self.FLAGS |= 0x01
        
        # 设置符号标志
        if result & 0x80:
            self.FLAGS |= 0x04
        else:
            self.FLAGS &= 0xFB  # 清除符号标志
        
        return True
    
    def in_(self):
        """从输入读取到A (隐含寻址)"""
        try:
            self.A = int(input("输入一个字节 (0-255): ")) & 0xFF
            self.update_flags(self.A)
        except ValueError:
            print("无效输入")
        return True
    
    # 中断指令
    def ei(self):
        """允许中断 (隐含寻址)"""
        self.interrupt_enabled = True
        self.FLAGS |= 0x08  # 设置中断允许标志
        print("中断已允许")
        return True
    
    def di(self):
        """禁止中断 (隐含寻址)"""
        self.interrupt_enabled = False
        self.FLAGS &= 0xF7  # 清除中断允许标志
        print("中断已禁止")
        return True
    
    def iret(self):
        """从中断返回 (隐含寻址)"""
        if self.SP > 3:
            # 弹出保存的优先级
            self.SP -= 1
            saved_priority = self.stack[self.SP]
            
            # 弹出FLAGS
            self.SP -= 1
            self.FLAGS = self.stack[self.SP]
            
            # 弹出PC高8位
            self.SP -= 1
            high_byte = self.stack[self.SP]
            
            # 弹出PC低8位
            self.SP -= 1
            low_byte = self.stack[self.SP]
            
            self.PC = (high_byte << 8) | low_byte
            
            # 恢复中断优先级
            self.current_priority = saved_priority
            self.nesting_level -= 1
            
            # 根据FLAGS恢复中断允许状态
            self.interrupt_enabled = bool(self.FLAGS & 0x08)
            
            print(f"从中断返回，PC=0x{self.PC:04X}, 优先级={saved_priority}, 嵌套层级={self.nesting_level}")
        else:
            print("栈下溢")
        return True
    
    def int(self):
        """软件中断 (直接寻址)"""
        vector = self.fetch_byte()
        self.trigger_interrupt(vector)
        return True
    
    # I/O指令
    def in_io(self):
        """从I/O端口读取到A (直接寻址)"""
        addr = self.fetch_byte()
        self.A = self.read_io(addr)
        self.update_flags(self.A)
        print(f"从I/O端口 0x{addr:02X} 读取: 0x{self.A:02X}")
        return True
    
    def out_io(self):
        """从A输出到I/O端口 (直接寻址)"""
        addr = self.fetch_byte()
        self.write_io(addr, self.A)
        print(f"向I/O端口 0x{addr:02X} 写入: 0x{self.A:02X}")
        return True
    
    # 变址寄存器指令
    def ldx(self):
        """加载立即数到IX (立即寻址)"""
        self.IX = self.fetch_byte()
        return True
    
    def stx(self):
        """存储IX到内存 (直接寻址)"""
        addr = self.fetch_byte()
        self.memory[addr] = self.IX
        return True
    
    def inx(self):
        """IX递增 (隐含寻址)"""
        self.IX = (self.IX + 1) & 0xFF
        return True
    
    def dcx(self):
        """IX递减 (隐含寻址)"""
        self.IX = (self.IX - 1) & 0xFF
        return True
    
    # 寻址模式指令
    def lda_direct(self):
        """LDA 直接寻址: 从内存地址加载到A"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            self.A = self.read_io(addr)
        else:
            self.A = self.memory[addr]
        self.update_flags(self.A)
        print(f"LDA直接寻址: 地址 0x{addr:02X}, 值 0x{self.A:02X}")
        return True
    
    def lda_indirect(self):
        """LDA 间接寻址: 从内存地址指向的地址加载到A"""
        ptr_addr = self.fetch_byte()
        addr = self.memory[ptr_addr]
        if addr in self.io_map:
            self.A = self.read_io(addr)
        else:
            self.A = self.memory[addr]
        self.update_flags(self.A)
        print(f"LDA间接寻址: 指针地址 0x{ptr_addr:02X}, 目标地址 0x{addr:02X}, 值 0x{self.A:02X}")
        return True
    
    def lda_indexed(self):
        """LDA 变址寻址: 从基址加变址寄存器加载到A"""
        base_addr = self.fetch_byte()
        addr = (base_addr + self.IX) & 0xFF
        if addr in self.io_map:
            self.A = self.read_io(addr)
        else:
            self.A = self.memory[addr]
        self.update_flags(self.A)
        print(f"LDA变址寻址: 基址 0x{base_addr:02X}, IX={self.IX}, 地址 0x{addr:02X}, 值 0x{self.A:02X}")
        return True
    
    def sta_direct(self):
        """STA 直接寻址: 存储A到内存地址"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            self.write_io(addr, self.A)
        else:
            self.memory[addr] = self.A
        print(f"STA直接寻址: 地址 0x{addr:02X}, 值 0x{self.A:02X}")
        return True
    
    def sta_indirect(self):
        """STA 间接寻址: 存储A到内存地址指向的地址"""
        ptr_addr = self.fetch_byte()
        addr = self.memory[ptr_addr]
        if addr in self.io_map:
            self.write_io(addr, self.A)
        else:
            self.memory[addr] = self.A
        print(f"STA间接寻址: 指针地址 0x{ptr_addr:02X}, 目标地址 0x{addr:02X}, 值 0x{self.A:02X}")
        return True
    
    def sta_indexed(self):
        """STA 变址寻址: 存储A到基址加变址寄存器"""
        base_addr = self.fetch_byte()
        addr = (base_addr + self.IX) & 0xFF
        if addr in self.io_map:
            self.write_io(addr, self.A)
        else:
            self.memory[addr] = self.A
        print(f"STA变址寻址: 基址 0x{base_addr:02X}, IX={self.IX}, 地址 0x{addr:02X}, 值 0x{self.A:02X}")
        return True
    
    def add_direct(self):
        """ADD 直接寻址: A = A + [内存地址]"""
        addr = self.fetch_byte()
        if addr in self.io_map:
            value = self.read_io(addr)
        else:
            value = self.memory[addr]
        
        result = self.A + value
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置进位标志
        if result > 0xFF:
            self.FLAGS |= 0x01  # 设置进位标志
            result &= 0xFF  # 截断到8位
        
        self.A = result
        self.update_flags(self.A)
        print(f"ADD直接寻址: 地址 0x{addr:02X}, 值 0x{value:02X}, 结果 0x{self.A:02X}")
        return True
    
    def add_indirect(self):
        """ADD 间接寻址: A = A + [[指针地址]]"""
        ptr_addr = self.fetch_byte()
        addr = self.memory[ptr_addr]
        if addr in self.io_map:
            value = self.read_io(addr)
        else:
            value = self.memory[addr]
        
        result = self.A + value
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置进位标志
        if result > 0xFF:
            self.FLAGS |= 0x01  # 设置进位标志
            result &= 0xFF  # 截断到8位
        
        self.A = result
        self.update_flags(self.A)
        print(f"ADD间接寻址: 指针地址 0x{ptr_addr:02X}, 目标地址 0x{addr:02X}, 值 0x{value:02X}, 结果 0x{self.A:02X}")
        return True
    
    def add_indexed(self):
        """ADD 变址寻址: A = A + [基址 + IX]"""
        base_addr = self.fetch_byte()
        addr = (base_addr + self.IX) & 0xFF
        if addr in self.io_map:
            value = self.read_io(addr)
        else:
            value = self.memory[addr]
        
        result = self.A + value
        self.FLAGS &= 0xFE  # 清除进位标志
        
        # 设置进位标志
        if result > 0xFF:
            self.FLAGS |= 0x01  # 设置进位标志
            result &= 0xFF  # 截断到8位
        
        self.A = result
        self.update_flags(self.A)
        print(f"ADD变址寻址: 基址 0x{base_addr:02X}, IX={self.IX}, 地址 0x{addr:02X}, 值 0x{value:02X}, 结果 0x{self.A:02X}")
        return True
    
    def jmp_indirect(self):
        """JMP 间接寻址: 跳转到内存地址指向的地址"""
        ptr_addr = self.fetch_byte()
        self.PC = self.memory[ptr_addr]
        print(f"JMP间接寻址: 指针地址 0x{ptr_addr:02X}, 目标地址 0x{self.PC:02X}")
        return True
    
    def jmp_indexed(self):
        """JMP 变址寻址: 跳转到基址加变址寄存器"""
        base_addr = self.fetch_byte()
        self.PC = (base_addr + self.IX) & 0xFF
        print(f"JMP变址寻址: 基址 0x{base_addr:02X}, IX={self.IX}, 目标地址 0x{self.PC:02X}")
        return True
    
    # 浮点指令
    def fld(self):
        """加载浮点数到F0"""
        addr = self.fetch_byte()
        # 从内存中读取4个字节
        bytes_data = bytes(self.memory[addr:addr+4])
        self.F0 = struct.unpack('f', bytes_data)[0]
        print(f"FLD: 从地址 0x{addr:02X} 加载浮点数 {self.F0}")
        return True
    
    def fst(self):
        """存储F0到内存"""
        addr = self.fetch_byte()
        # 将浮点数转换为4个字节
        bytes_data = struct.pack('f', self.F0)
        for i, byte in enumerate(bytes_data):
            self.memory[addr + i] = byte
        print(f"FST: 存储浮点数 {self.F0} 到地址 0x{addr:02X}")
        return True
    
    def fadd(self):
        """浮点加法: F0 = F0 + F1"""
        self.F0 += self.F1
        print(f"FADD: F0 = {self.F0}, F1 = {self.F1}")
        return True
    
    def fsub(self):
        """浮点减法: F0 = F0 - F1"""
        self.F0 -= self.F1
        print(f"FSUB: F0 = {self.F0}, F1 = {self.F1}")
        return True
    
    def fmul(self):
        """浮点乘法: F0 = F0 * F1"""
        self.F0 *= self.F1
        print(f"FMUL: F0 = {self.F0}, F1 = {self.F1}")
        return True
    
    def fdiv(self):
        """浮点除法: F0 = F0 / F1"""
        if self.F1 == 0.0:
            print("错误: 浮点除零")
            return False
        self.F0 /= self.F1
        print(f"FDIV: F0 = {self.F0}, F1 = {self.F1}")
        return True
    
    def fsqrt(self):
        """浮点平方根: F0 = sqrt(F0)"""
        if self.F0 < 0.0:
            print("错误: 负数平方根")
            return False
        self.F0 = math.sqrt(self.F0)
        print(f"FSQRT: F0 = {self.F0}")
        return True
    
    def fsin(self):
        """浮点正弦: F0 = sin(F0)"""
        self.F0 = math.sin(self.F0)
        print(f"FSIN: F0 = {self.F0}")
        return True
    
    def fcos(self):
        """浮点余弦: F0 = cos(F0)"""
        self.F0 = math.cos(self.F0)
        print(f"FCOS: F0 = {self.F0}")
        return True
    
    def ftan(self):
        """浮点正切: F0 = tan(F0)"""
        self.F0 = math.tan(self.F0)
        print(f"FTAN: F0 = {self.F0}")
        return True
    
    def fcmp(self):
        """浮点比较: 比较F0和F1"""
        if self.F0 == self.F1:
            self.FLAGS |= 0x02  # 设置零标志
        else:
            self.FLAGS &= 0xFD  # 清除零标志
        
        if self.F0 < self.F1:
            self.FLAGS |= 0x01  # 设置进位标志 (表示小于)
        else:
            self.FLAGS &= 0xFE  # 清除进位标志
        
        print(f"FCMP: F0 = {self.F0}, F1 = {self.F1}, 标志 = {self.FLAGS:02X}")
        return True
    
    def fmov(self):
        """浮点移动: F0 = F1"""
        self.F0 = self.F1
        print(f"FMOV: F0 = {self.F0}, F1 = {self.F1}")
        return True
    
    def fpush(self):
        """浮点压栈"""
        if self.fp_sp < len(self.fp_stack):
            self.fp_stack[self.fp_sp] = self.F0
            self.fp_sp += 1
            print(f"FPUSH: F0 = {self.F0}, FP_SP = {self.fp_sp}")
        else:
            print("浮点栈溢出")
        return True
    
    def fpop(self):
        """浮点出栈"""
        if self.fp_sp > 0:
            self.fp_sp -= 1
            self.F0 = self.fp_stack[self.fp_sp]
            print(f"FPOP: F0 = {self.F0}, FP_SP = {self.fp_sp}")
        else:
            print("浮点栈下溢")
        return True
    
    def fint(self):
        """浮点转整数: A = (int)F0"""
        self.A = int(self.F0) & 0xFF
        self.update_flags(self.A)
        print(f"FINT: F0 = {self.F0}, A = {self.A}")
        return True
    
    def ffloat(self):
        """整数转浮点: F0 = (float)A"""
        self.F0 = float(self.A)
        print(f"FFLOAT: A = {self.A}, F0 = {self.F0}")
        return True
    
    def fout(self):
        """输出浮点数F0"""
        print(f"浮点输出: {self.F0}")
        return True
    
    def dump_registers(self):
        """打印寄存器状态"""
        print(f"A: 0x{self.A:02X} ({self.A})")
        print(f"B: 0x{self.B:02X} ({self.B})")
        print(f"C: 0x{self.C:02X} ({self.C})")
        print(f"D: 0x{self.D:02X} ({self.D})")
        print(f"PC: 0x{self.PC:04X}")
        print(f"SP: 0x{self.SP:02X}")
        print(f"IX: 0x{self.IX:02X} ({self.IX})")
        print(f"FLAGS: 0x{self.FLAGS:02X}")
        flags = []
        if self.FLAGS & 0x01: flags.append("进位")
        if self.FLAGS & 0x02: flags.append("零")
        if self.FLAGS & 0x04: flags.append("符号")
        if self.FLAGS & 0x08: flags.append("中断允许")
        print(f"标志位: {', '.join(flags) if flags else '无'}")
        print(f"中断允许: {'是' if self.interrupt_enabled else '否'}")
        print(f"当前中断优先级: {self.current_priority}")
        print(f"中断嵌套层级: {self.nesting_level}")
        print(f"待处理中断数: {self.interrupt_queue.qsize()}")
        
        # 打印浮点寄存器
        print(f"F0: {self.F0}")
        print(f"F1: {self.F1}")
        print(f"F2: {self.F2}")
        print(f"F3: {self.F3}")
        print(f"FP_SP: {self.fp_sp}")
        
        # 打印栈内容
        print("栈内容 (从底到顶):")
        if self.SP > 0:
            for i in range(min(self.SP, 8)):  # 只显示前8个栈元素
                print(f"  [{i}]: 0x{self.stack[i]:02X} ({self.stack[i]})")
            if self.SP > 8:
                print(f"  ... ({self.SP - 8} 更多)")
        else:
            print("  空")
        
        # 打印浮点栈内容
        print("浮点栈内容 (从底到顶):")
        if self.fp_sp > 0:
            for i in range(min(self.fp_sp, 4)):  # 只显示前4个浮点栈元素
                print(f"  [{i}]: {self.fp_stack[i]}")
            if self.fp_sp > 4:
                print(f"  ... ({self.fp_sp - 4} 更多)")
        else:
            print("  空")
    
    def dump_io_devices(self):
        """打印I/O设备状态"""
        print("\nI/O设备状态:")
        for addr in sorted(self.io_map.keys()):
            device_name = self.io_map[addr]
            value = self.io_devices[device_name]
            print(f"  0x{addr:02X} ({device_name}): 0x{value:02X} ({value})")

# 示例程序: 浮点运算演示
float_program = [
    # 加载浮点数到内存
    0x01, 0x90,        # LDA #0x90 (浮点数1的地址)
    0x05, 0xA0,        # STA 0xA0 (存储指针)
    0x01, 0x94,        # LDA #0x94 (浮点数2的地址)
    0x05, 0xA1,        # STA 0xA1 (存储指针)
    
    # 加载浮点数
    0x50, 0x90,        # FLD 0x90 (加载浮点数3.14)
    0x51, 0x98,        # FST 0x98 (存储到临时位置)
    0x50, 0x94,        # FLD 0x94 (加载浮点数2.71)
    0x5B,              # FMOV (F1 = F0)
    0x50, 0x98,        # FLD 0x98 (加载之前存储的3.14)
    
    # 浮点运算
    0x52,              # FADD (F0 = F0 + F1 = 3.14 + 2.71)
    0x51, 0x9C,        # FST 0x9C (存储加法结果)
    0x60,              # FOUT (输出结果)
    
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x5B,              # FMOV (F1 = F0)
    0x50, 0x94,        # FLD 0x94 (加载2.71)
    0x53,              # FSUB (F0 = F0 - F1 = 2.71 - 3.14)
    0x60,              # FOUT (输出结果)
    
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x5B,              # FMOV (F1 = F0)
    0x50, 0x94,        # FLD 0x94 (加载2.71)
    0x54,              # FMUL (F0 = F0 * F1 = 3.14 * 2.71)
    0x60,              # FOUT (输出结果)
    
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x5B,              # FMOV (F1 = F0)
    0x50, 0x94,        # FLD 0x94 (加载2.71)
    0x55,              # FDIV (F0 = F0 / F1 = 2.71 / 3.14)
    0x60,              # FOUT (输出结果)
    
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x56,              # FSQRT (F0 = sqrt(3.14))
    0x60,              # FOUT (输出结果)
    
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x57,              # FSIN (F0 = sin(3.14))
    0x60,              # FOUT (输出结果)
    
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x58,              # FCOS (F0 = cos(3.14))
    0x60,              # FOUT (输出结果)
    
    # 浮点转整数
    0x50, 0x90,        # FLD 0x90 (加载3.14)
    0x5E,              # FINT (A = (int)3.14)
    0x16,              # OUT (输出整数部分)
    
    0x00,              # HLT
]

# 浮点数数据 (3.14 和 2.71 的IEEE 754单精度表示)
# 3.14 = 0x4048F5C3
# 2.71 = 0x402D70A4
float_data = [
    0x40, 0x48, 0xF5, 0xC3,  # 3.14 (地址 0x90-0x93)
    0x40, 0x2D, 0x70, 0xA4,  # 2.71 (地址 0x94-0x97)
    0x00, 0x00, 0x00, 0x00,  # 临时存储 (地址 0x98-0x9B)
    0x00, 0x00, 0x00, 0x00,  # 结果存储 (地址 0x9C-0x9F)
    0x00, 0x00, 0x00, 0x00,  # 指针存储 (地址 0xA0-0xA3)
]

if __name__ == "__main__":
    # 创建CPU实例
    cpu = FloatCPUSimulator()
    
    # 加载程序到内存
    cpu.load_program(float_program, 0x60)
    
    # 加载浮点数数据
    cpu.load_program(float_data, 0x90)
    
    # 执行程序
    print("开始执行浮点运算程序...")
    
    # 执行程序
    cpu.execute(max_cycles=100)
    
    # 打印寄存器状态
    print("\n最终寄存器状态:")
    cpu.dump_registers()
    
    # 打印内存中存储的结果
    print(f"\n内存中存储的浮点数:")
    for addr in range(0x90, 0xA0, 4):
        bytes_data = bytes(cpu.memory[addr:addr+4])
        value = struct.unpack('f', bytes_data)[0]
        print(f"地址 0x{addr:02X}-0x{addr+3:02X}: {value}")