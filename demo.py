import threading
import time
import queue

class PriorityInterruptCPU:
    def __init__(self, memory_size=256, stack_size=16):
        # 8位寄存器
        self.A = 0  # 累加器
        self.B = 0  # 通用寄存器
        self.C = 0  # 通用寄存器
        self.D = 0  # 通用寄存器
        self.PC = 0  # 程序计数器
        self.SP = 0  # 栈指针 (指向栈顶)
        self.FLAGS = 0  # 标志寄存器 (Z:零标志, C:进位标志, S:符号标志, I:中断允许标志)
        
        # 内存 (256字节)
        self.memory = [0] * memory_size
        
        # 栈 (16字节)
        self.stack = [0] * stack_size
        self.stack_size = stack_size
        
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
        self.memory[0x40] = 0x05  # STA
        self.memory[0x41] = 0xF2  # 存储A到地址 0xF2 (定时器值)
        self.memory[0x42] = 0x0D  # INC (递增定时器)
        self.memory[0x43] = 0x34  # IRET
        
        # 中断3处理程序: 低优先级中断 (模拟I/O中断)
        self.memory[0x50] = 0x31  # IN
        self.memory[0x51] = 0x05  # STA
        self.memory[0x52] = 0xF3  # 存储到地址 0xF3 (输入数据)
        self.memory[0x53] = 0x34  # IRET
    
    def reset(self):
        """重置CPU状态"""
        self.A = 0
        self.B = 0
        self.C = 0
        self.D = 0
        self.PC = 0
        self.SP = 0
        self.FLAGS = 0
        self.memory = [0] * len(self.memory)
        self.stack = [0] * self.stack_size
        
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
    
    def update_flags(self, value):
        """根据值更新标志寄存器"""
        self.FLAGS &= 0xF9  # 清除零标志和符号标志，保留进位标志和中断允许标志
        
        # 设置零标志
        if value == 0:
            self.FLAGS |= 0x02  # 设置零标志
        
        # 设置符号标志 (最高位)
        if value & 0x80:
            self.FLAGS |= 0x04  # 设置符号标志
    
    # 指令实现
    def hlt(self):
        """停止执行"""
        return False
    
    def lda(self):
        """加载立即数到A"""
        self.A = self.fetch_byte()
        self.update_flags(self.A)
        return True
    
    def ldb(self):
        """加载立即数到B"""
        self.B = self.fetch_byte()
        return True
    
    def ldc(self):
        """加载立即数到C"""
        self.C = self.fetch_byte()
        return True
    
    def ldd(self):
        """加载立即数到D"""
        self.D = self.fetch_byte()
        return True
    
    def sta(self):
        """存储A到内存"""
        addr = self.fetch_byte()
        self.memory[addr] = self.A
        return True
    
    def stb(self):
        """存储B到内存"""
        addr = self.fetch_byte()
        self.memory[addr] = self.B
        return True
    
    def stc(self):
        """存储C到内存"""
        addr = self.fetch_byte()
        self.memory[addr] = self.C
        return True
    
    def std(self):
        """存储D到内存"""
        addr = self.fetch_byte()
        self.memory[addr] = self.D
        return True
    
    def add(self):
        """A = A + B"""
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
        """A = A - B"""
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
        """A = A * B"""
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
        """A = A / B"""
        if self.B == 0:
            print("错误: 除以零")
            return False
        
        result = self.A // self.B
        self.A = result
        self.update_flags(self.A)
        return True
    
    def inc(self):
        """A = A + 1"""
        self.A = (self.A + 1) & 0xFF
        self.update_flags(self.A)
        return True
    
    def dec(self):
        """A = A - 1"""
        self.A = (self.A - 1) & 0xFF
        self.update_flags(self.A)
        return True
    
    def jmp(self):
        """跳转到地址"""
        self.PC = self.fetch_byte()
        return True
    
    def jz(self):
        """如果零标志为真则跳转"""
        addr = self.fetch_byte()
        if self.FLAGS & 0x02:  # 检查零标志
            self.PC = addr
        return True
    
    def jnz(self):
        """如果零标志为假则跳转"""
        addr = self.fetch_byte()
        if not (self.FLAGS & 0x02):  # 检查零标志
            self.PC = addr
        return True
    
    def jc(self):
        """如果进位标志为真则跳转"""
        addr = self.fetch_byte()
        if self.FLAGS & 0x01:  # 检查进位标志
            self.PC = addr
        return True
    
    def jnc(self):
        """如果进位标志为假则跳转"""
        addr = self.fetch_byte()
        if not (self.FLAGS & 0x01):  # 检查进位标志
            self.PC = addr
        return True
    
    def js(self):
        """如果符号标志为真则跳转"""
        addr = self.fetch_byte()
        if self.FLAGS & 0x04:  # 检查符号标志
            self.PC = addr
        return True
    
    def jns(self):
        """如果符号标志为假则跳转"""
        addr = self.fetch_byte()
        if not (self.FLAGS & 0x04):  # 检查符号标志
            self.PC = addr
        return True
    
    def out(self):
        """输出A的值"""
        print(f"输出: {self.A} (0x{self.A:02X})")
        return True
    
    def nop(self):
        """空操作"""
        return True
    
    def and_(self):
        """A = A & B"""
        self.A &= self.B
        self.update_flags(self.A)
        return True
    
    def or_(self):
        """A = A | B"""
        self.A |= self.B
        self.update_flags(self.A)
        return True
    
    def xor(self):
        """A = A ^ B"""
        self.A ^= self.B
        self.update_flags(self.A)
        return True
    
    def not_(self):
        """A = ~A"""
        self.A = (~self.A) & 0xFF
        self.update_flags(self.A)
        return True
    
    def shl(self):
        """算术左移: A << 1, 最高位进入进位标志"""
        carry = (self.A & 0x80) >> 7  # 获取最高位
        self.A = (self.A << 1) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def shr(self):
        """算术右移: A >> 1, 最低位进入进位标志"""
        carry = self.A & 0x01  # 获取最低位
        self.A = (self.A >> 1) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def rol(self):
        """循环左移: 进位标志参与循环"""
        carry = (self.A & 0x80) >> 7  # 获取最高位
        self.A = ((self.A << 1) | (self.FLAGS & 0x01)) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def ror(self):
        """循环右移: 进位标志参与循环"""
        carry = self.A & 0x01  # 获取最低位
        self.A = ((self.A >> 1) | ((self.FLAGS & 0x01) << 7)) & 0xFF
        
        self.FLAGS &= 0xFE  # 清除进位标志
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def push(self, value=None):
        """将值压入栈"""
        if value is None:
            value = self.A
        
        if self.SP < self.stack_size:
            self.stack[self.SP] = value
            self.SP += 1
        else:
            print("栈溢出")
        return True
    
    def pop(self):
        """从栈弹出到A"""
        if self.SP > 0:
            self.SP -= 1
            self.A = self.stack[self.SP]
            self.update_flags(self.A)
        else:
            print("栈下溢")
        return True
    
    def call(self):
        """调用子程序: 将返回地址压栈并跳转"""
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
        """从子程序返回: 从栈中弹出返回地址"""
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
        """MOV A, B"""
        self.A = self.B
        self.update_flags(self.A)
        return True
    
    def mov_ac(self):
        """MOV A, C"""
        self.A = self.C
        self.update_flags(self.A)
        return True
    
    def mov_ad(self):
        """MOV A, D"""
        self.A = self.D
        self.update_flags(self.A)
        return True
    
    def mov_ba(self):
        """MOV B, A"""
        self.B = self.A
        return True
    
    def mov_bc(self):
        """MOV B, C"""
        self.B = self.C
        return True
    
    def mov_bd(self):
        """MOV B, D"""
        self.B = self.D
        return True
    
    def mov_ca(self):
        """MOV C, A"""
        self.C = self.A
        return True
    
    def mov_cb(self):
        """MOV C, B"""
        self.C = self.B
        return True
    
    def mov_cd(self):
        """MOV C, D"""
        self.C = self.D
        return True
    
    def mov_da(self):
        """MOV D, A"""
        self.D = self.A
        return True
    
    def mov_db(self):
        """MOV D, B"""
        self.D = self.B
        return True
    
    def mov_dc(self):
        """MOV D, C"""
        self.D = self.C
        return True
    
    def cmp(self):
        """比较A和B，设置标志位"""
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
        """从输入读取到A"""
        try:
            self.A = int(input("输入一个字节 (0-255): ")) & 0xFF
            self.update_flags(self.A)
        except ValueError:
            print("无效输入")
        return True
    
    # 中断指令
    def ei(self):
        """允许中断"""
        self.interrupt_enabled = True
        self.FLAGS |= 0x08  # 设置中断允许标志
        print("中断已允许")
        return True
    
    def di(self):
        """禁止中断"""
        self.interrupt_enabled = False
        self.FLAGS &= 0xF7  # 清除中断允许标志
        print("中断已禁止")
        return True
    
    def iret(self):
        """从中断返回"""
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
        """软件中断"""
        vector = self.fetch_byte()
        self.trigger_interrupt(vector)
        return True
    
    def dump_registers(self):
        """打印寄存器状态"""
        print(f"A: 0x{self.A:02X} ({self.A})")
        print(f"B: 0x{self.B:02X} ({self.B})")
        print(f"C: 0x{self.C:02X} ({self.C})")
        print(f"D: 0x{self.D:02X} ({self.D})")
        print(f"PC: 0x{self.PC:04X}")
        print(f"SP: 0x{self.SP:02X}")
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
        
        # 打印栈内容
        print("栈内容 (从底到顶):")
        if self.SP > 0:
            for i in range(min(self.SP, 8)):  # 只显示前8个栈元素
                print(f"  [{i}]: 0x{self.stack[i]:02X} ({self.stack[i]})")
            if self.SP > 8:
                print(f"  ... ({self.SP - 8} 更多)")
        else:
            print("  空")

# 示例程序: 主程序，等待中断
main_program = [
    0x32,        # EI (允许中断)
    0x01, 0x00,  # LDA 0x00
    0x0D,        # INC (递增A)
    0x07, 0x02,  # JMP 0x0002 (循环)
]

if __name__ == "__main__":
    # 创建CPU实例
    cpu = PriorityInterruptCPU()
    
    # 加载程序到内存
    cpu.load_program(main_program, 0x60)
    
    # 执行程序
    print("开始执行程序...")
    
    # 模拟中断的线程
    def simulate_interrupts():
        # 先触发低优先级中断
        time.sleep(0.5)
        print("\n触发低优先级中断 (I/O)...")
        cpu.trigger_interrupt(3)  # 低优先级中断
        
        # 然后触发高优先级中断，应该会打断低优先级中断
        time.sleep(0.5)
        print("\n触发高优先级中断 (硬件故障)...")
        cpu.trigger_interrupt(1)  # 高优先级中断
        
        # 最后触发最高优先级中断，应该会打断所有中断
        time.sleep(0.5)
        print("\n触发最高优先级中断 (系统复位)...")
        cpu.trigger_interrupt(0)  # 最高优先级中断
        
        time.sleep(1)
        print("\n停止模拟")
        # 停止CPU执行
        cpu.memory[cpu.PC] = 0x00  # 将下一条指令改为HLT
    
    # 启动中断模拟线程
    interrupt_thread = threading.Thread(target=simulate_interrupts)
    interrupt_thread.daemon = True
    interrupt_thread.start()
    
    # 执行程序
    cpu.execute(max_cycles=100)
    
    # 打印寄存器状态
    print("\n最终寄存器状态:")
    cpu.dump_registers()
    
    # 打印内存中存储的结果
    print(f"\n内存中存储的结果:")
    print(f"地址 0xF0: {cpu.memory[0xF0]} (系统状态)")
    print(f"地址 0xF1: {cpu.memory[0xF1]} (错误代码)")
    print(f"地址 0xF2: {cpu.memory[0xF2]} (定时器值)")
    print(f"地址 0xF3: {cpu.memory[0xF3]} (输入数据)")