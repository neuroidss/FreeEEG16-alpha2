import serial
import time
from pylsl import StreamInfo, StreamOutlet

# ================= НАСТРОЙКИ =================
SERIAL_PORT = '/dev/ttyACM0'  # Укажите ваш порт (например, COM3 или /dev/ttyACM0)
BAUD_RATE = 2000000
CHANNELS_PER_NODE = 16
PACKET_SIZE = 57  # 1(A0) + 1(Count) + 48(Data) + 6(MAC) + 1(C0)
# =============================================

class Node:
    def __init__(self, mac_str):
        self.mac_str = mac_str
        self.last_counter = -1
        self.packets_received = 0
        self.packets_lost = 0
        
        clean_mac = self.mac_str.replace(":", "").replace("-", "")
        print(f"[NEW DEVICE DETECTED] MAC: {mac_str}. Creating LSL Stream 'FreeEEG_{clean_mac}'...")
        info = StreamInfo(f'FreeEEG_{clean_mac}', 'EEG', CHANNELS_PER_NODE, 
                          0, 'int32', f'uid_{mac_str}')
        self.outlet = StreamOutlet(info)

active_nodes = {}

def parse_24bit_to_int32(raw_bytes):
    channels = []
    for i in range(16):
        idx = i * 3
        val = (raw_bytes[idx] << 16) | (raw_bytes[idx+1] << 8) | raw_bytes[idx+2]
        if val & 0x800000:
            val -= 0x1000000
        channels.append(val)
    return channels

def main():
    print(f"Connecting to Dongle on {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    
    # УДАЛЕНО: ser.set_buffer_size (не поддерживается в Linux)
    
    buffer = bytearray()
    last_print_time = time.time()
    
    print("Listening for Dongle serial stream... Press Ctrl+C to stop.")
    
    while True:
        # Читаем данные из USB крупными пачками (в 10 раз быстрее побайтового чтения)
        chunk = ser.read(4096)
        if chunk:
            buffer.extend(chunk)
            
        while len(buffer) >= PACKET_SIZE:
            # Проверяем маркеры пакета
            if buffer[0] == 0xA0 and buffer[PACKET_SIZE - 1] == 0xC0:
                counter = buffer[1]
                adc_data = buffer[2:50]
                mac_bytes = buffer[50:56]
                
                mac_str = ':'.join(f'{b:02X}' for b in mac_bytes)
                
                if mac_str not in active_nodes:
                    active_nodes[mac_str] = Node(mac_str)
                    
                node = active_nodes[mac_str]
                
                # Считаем потери по счетчику
                if node.last_counter != -1:
                    expected_counter = (node.last_counter + 1) % 256
                    if counter != expected_counter:
                        loss = (counter - expected_counter) % 256
                        node.packets_lost += loss
                        
                node.last_counter = counter
                node.packets_received += 1
                
                # Парсинг и отправка в LSL
                channel_data = parse_24bit_to_int32(adc_data)
                node.outlet.push_sample(channel_data)
                
                # Удаляем обработанный кусок буфера
                del buffer[:PACKET_SIZE]
            else:
                # Если произошел сдвиг, мгновенно прыгаем к следующему началу пакета
                next_a0 = buffer.find(0xA0, 1)
                if next_a0 == -1:
                    del buffer[:]
                else:
                    del buffer[:next_a0]

        # Ежесекундная статистика
        current_time = time.time()
        if current_time - last_print_time >= 1.0:
            if active_nodes:
                print("\n--- Dongle LSL Bridge Stats (Last 1 Second) ---")
                for mac, n in active_nodes.items():
                    print(f"Device [{mac}]: Rx = {n.packets_received} pkts/s | Lost Total = {n.packets_lost}")
                    n.packets_received = 0 
            last_print_time = current_time

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
