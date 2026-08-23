import random

rnd_num = random.randint(1,100)
game_cnt = 1

def check_num(my_num)->bool:
    if my_num > rnd_num :
        print("Down")
        return False
    elif my_num < rnd_num :
        print("Up")
        return False
    elif my_num == rnd_num :
        print(f"Conglaturations!", end=' ')
        return True

if __name__ == '__main__' :
    while True :
        try :
            num = int(input("Input number 1~99 :"))
            if check_num(num) :
                print(f'you got it right in {game_cnt} tries.')            
                break
            game_cnt += 1
        except Exception as e:
            print(e)