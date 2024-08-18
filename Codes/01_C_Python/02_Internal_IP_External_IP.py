import socket
import urllib.request

def get_internal_ip():
    try:
        host_name = socket.gethostname()
        internal_ip = socket.gethostbyname(host_name)
        return internal_ip
    except Exception as e:
        return str(e)

def get_external_ip():
    try:
        url = "https://api64.ipify.org?format=json"
        response = urllib.request.urlopen(url)
        data = response.read().decode("utf-8")
        external_ip = data.split(':')[1].split('"')[1]
        return external_ip
    except Exception as e:
        return str(e)

internal_ip = get_internal_ip()
external_ip = get_external_ip()

print("Int. IP :", internal_ip)
print("Ext. IP :", external_ip)