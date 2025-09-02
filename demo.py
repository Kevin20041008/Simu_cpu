class ExtendedCPU:
    def __init__(self, memory_size=256, stack_size=16):
        # 8位寄存器
        self.A = 0  # 累加器
        self.B = 0  # 通用寄存器
        self.C = 0  # 通用寄存器
        self.D = 0  # 通用寄存器
        self.PC = 0  # 程序计数器
        self.SP = 0  # 栈指针 (指向栈顶)
        self.FLAGS = 0  # 标志寄存器 (Z:零标志, C:进位标志, S:符号标志)
        
        # 内存 (256字节)
        self.memory = [0] * memory_size
        
        # 栈 (16字节)
        self.stack = [0] * stack_size
        self.stack_size = stack_size
        
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
        }
    
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
    
    def execute(self, max_cycles=1000):
        """执行程序"""
        running = True
        cycles = 0
        
        while running and cycles < max_cycles:
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
        self.FLAGS = 0  # 重置标志位
        
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
        self.FLAGS = 0  # 重置标志位
        
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
        self.FLAGS = 0  # 重置标志位
        
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
        self.FLAGS = 0  # 重置标志位
        
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
        
        self.FLAGS = 0
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def shr(self):
        """算术右移: A >> 1, 最低位进入进位标志"""
        carry = self.A & 0x01  # 获取最低位
        self.A = (self.A >> 1) & 0xFF
        
        self.FLAGS = 0
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def rol(self):
        """循环左移: 进位标志参与循环"""
        carry = (self.A & 0x80) >> 7  # 获取最高位
        self.A = ((self.A << 1) | (self.FLAGS & 0x01)) & 0xFF
        
        self.FLAGS = 0
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def ror(self):
        """循环右移: 进位标志参与循环"""
        carry = self.A & 0x01  # 获取最低位
        self.A = ((self.A >> 1) | ((self.FLAGS & 0x01) << 7)) & 0xFF
        
        self.FLAGS = 0
        if carry:
            self.FLAGS |= 0x01  # 设置进位标志
        
        self.update_flags(self.A)
        return True
    
    def push(self):
        """将A压入栈"""
        if self.SP < self.stack_size:
            self.stack[self.SP] = self.A
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
            # 压入返回地址高字节 (在8位CPU中，PC是8位，所以不需要分开高低字节)
            self.stack[self.SP] = return_addr
            self.SP += 1
            
            # 跳转到目标地址
            self.PC = self.fetch_byte()
        else:
            print("栈溢出")
        return True
    
    def ret(self):
        """从子程序返回: 从栈中弹出返回地址"""
        if self.SP > 0:
            self.SP -= 1
            self.PC = self.stack[self.SP]
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
        self.FLAGS = 0  # 重置标志位
        
        # 设置零标志
        if result == 0:
            self.FLAGS |= 0x02
        
        # 设置进位标志(借位)
        if result < 0:
            self.FLAGS |= 0x01
        
        # 设置符号标志
        if result & 0x80:
            self.FLAGS |= 0x04
        
        return True
    
    def in_(self):
        """从输入读取到A"""
        try:
            self.A = int(input("输入一个字节 (0-255): ")) & 0xFF
            self.update_flags(self.A)
        except ValueError:
            print("无效输入")
        return True
    
    def dump_registers(self):
        """打印寄存器状态"""
        print(f"A: 0x{self.A:02X} ({self.A})")
        print(f"B: 0x{self.B:02X} ({self.B})")
        print(f"C: 0x{self.C:02X} ({self.C})")
        print(f"D: 0x{self.D:02X} ({self.D})")
        print(f"PC: 0x{self.PC:02X}")
        print(f"SP: 0x{self.SP:02X}")
        print(f"FLAGS: 0x{self.FLAGS:02X}")
        flags = []
        if self.FLAGS & 0x01: flags.append("进位")
        if self.FLAGS & 0x02: flags.append("零")
        if self.FLAGS & 0x04: flags.append("符号")
        print(f"标志位: {', '.join(flags) if flags else '无'}")
        
        # 打印栈内容
        print("栈内容 (从底到顶):")
        if self.SP > 0:
            for i in range(min(self.SP, 8)):  # 只显示前8个栈元素
                print(f"  [{i}]: 0x{self.stack[i]:02X} ({self.stack[i]})")
            if self.SP > 8:
                print(f"  ... ({self.SP - 8} 更多)")
        else:
            print("  空")

# 示例程序: 计算阶乘
# 计算 5! = 120
factorial_program = [
    0x01, 0x05,  # LDA 5      ; 加载初始值
    0x02, 0x01,  # LDB 1      ; 加载乘数初始值
    0x24,        # MOV A, B   ; 保存当前值
    0x20,        # PUSH       ; 压栈保存
    
    # 循环开始
    0x21,        # POP        ; 弹出当前值
    0x27,        # MOV B, A   ; 移动到B
    0x0E,        # DEC        ; 递减
    0x20,        # PUSH       ; 压栈保存
    0x11, 0x0F,  # JNZ 0x0F   ; 如果不为零，继续循环
    
    # 结束
    0x21,        # POP        ; 弹出结果
    0x16,        # OUT        ; 输出结果
    0x00         # HLT        ; 停止
]

# 简单测试程序: 计算 10 + 20
simple_program = [
    0x01, 10,    # LDA 10
    0x02, 20,    # LDB 20
    0x09,        # ADD
    0x16,        # OUT
    0x00         # HLT
]

if __name__ == "__main__":
    # 创建CPU实例
    cpu = ExtendedCPU()
    
    # 加载程序到内存
    cpu.load_program(simple_program)
    
    # 执行程序
    cpu.execute()
    
    # 打印寄存器状态
    print("\n最终寄存器状态:")
    cpu.dump_registers()