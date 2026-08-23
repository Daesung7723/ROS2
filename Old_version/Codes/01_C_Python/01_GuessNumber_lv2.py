import random

def menu() -> int:
    try:
        num = int(input('Select num\n1. Game\n2. Status\n3. Exit\n'))
        return num
    except Exception as e :
        print(e)
        return 0

def Guess() -> list:
    g_num = []
    g_num.append(int(input('1st num : ')))
    g_num.append(int(input('2nd num : ')))
    return g_num


def Game() -> int:
    rtn_cnt = 0

    rnd_num = []
    rnd_num.append(random.randint(1,9))
    rnd_num.append(random.randint(1,9))
    while rnd_num[0] is rnd_num[1] :
        rnd_num.pop()
        rnd_num.append(random.randint(1,10))

    while True:
        strike = 0
        ball = 0
        rtn_cnt += 1
        guess_num = Guess()
        print(rnd_num)
        for i in range(len(rnd_num)) :
            if guess_num[i] == rnd_num[i]:
                strike += 1
            elif guess_num[i] in rnd_num :
                ball += 1
            else :
                pass
        
        print(f'Strike = {strike}, Ball = {ball}')
        if strike == 2 :
            return rtn_cnt
    


if __name__ == '__main__':
    while True:
        num = menu()
        if num == 3 :
            break
        elif num == 1 :
            cnt = Game()
        elif num == 2 :
            print(f'Try Counts : {cnt}')
        else :
            pass