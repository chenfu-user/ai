from raf.Node import App
import asyncio
from raf.SubBase import link
import time
import torch
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
import cv2



if __name__ == '__main__':
    app = App()

    async def main():
        n1 = app.create_node('ros')
        n2 = app.create_node('image_processing')

        await asyncio.sleep(0.1)  # 等待node创建完成

        link(n1.img_out, n2.img_in)
        link(n2.speed_out, n1.speed_in)

        await asyncio.sleep(0.1)  # 等待连接完成


    app.loop.run_until_complete(main())
    app.loop.run_forever()
    app.close()
