import asyncio
import telegram
token =
chat_id = 
message = '1st python test message'

async def main():
    bot = telegram.Bot(token)
    async with bot:
        await bot.send_message(text=message, chat_id=chat_id)

if __name__ == '__main__':
    asyncio.run(main())
