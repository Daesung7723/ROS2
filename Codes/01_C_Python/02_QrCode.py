import qrcode
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

qr_data = 'www.google.com'
qr_img = qrcode.make(qr_data)

save_path = 'QR Code_'+qr_data+'.png'
qr_img.save(save_path)
