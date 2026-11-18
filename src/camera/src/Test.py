# TmpList = [-70, -90, -30, -50, 100]

# SortedTmpList = sorted(TmpList, reverse=True) # 큰 -> 작

# print(SortedTmpList)
import numpy as np
import matplotlib.pyplot as plt

def plot_line(slope, intercept):
    # x 범위 설정
    x = np.linspace(-10, 10, 400)
    # y = 기울기 * x + y 절편
    y = slope * x + intercept
    
    # 그래프 그리기
    plt.figure(figsize=(6,6))
    plt.plot(x, y, '-r', label=f'y={slope}x+{intercept}')
    
    # x, y축 범위 설정
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    
    # 축 추가 및 제목 설정
    plt.axhline(0, color='black',linewidth=0.5)
    plt.axvline(0, color='black',linewidth=0.5)
    plt.title('Straight Line Plot')
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    plt.grid(True)
    plt.legend(loc='upper left')
    
    # 그래프 보여주기
    plt.show()

# 기울기와 y 절편을 넣어 직선을 그립니다
slope = float(input("Enter the slope: "))
intercept = float(input("Enter the y-intercept: "))
plot_line(slope, intercept)
