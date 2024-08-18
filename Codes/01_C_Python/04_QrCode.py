import qrcode
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

file_path = r'QrCodes.txt'

with open(file_path, 'rt', encoding='UTF8') as f :
    read_lines = f.readlines()
    cnt=0
    for line in read_lines :        
        cnt = cnt+1
        line = line.strip()
        print(line)
        qr_data = line
        qr_img = qrcode.make(qr_data)
        
        save_path = 'QR_Code'+ str(cnt) + '.png'
        qr_img.save(save_path)