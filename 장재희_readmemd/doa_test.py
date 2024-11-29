# 2024.11.28
# 음성이 감지된 각도(DOA 값) 출력

from tuning import Tuning
import usb.core
import usb.util
import time

def main():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    
    if dev:
        Mic_tuning = Tuning(dev)
        print("Starting voice detection with DOA (Direction of Arrival)...")
        
        try:
            while True:
                is_voice = Mic_tuning.is_voice()
                if is_voice:
                    doa = Mic_tuning.direction  # DOA 값을 읽음
                    print(f"Voice detected! Direction: {doa}°")
                else:
                    print("No voice detected.")
                time.sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user. Exiting.")
    else:
        print("ReSpeaker USB Mic Array not found. Please check the connection.")

if __name__ == "__main__":
    main()