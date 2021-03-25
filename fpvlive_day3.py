# -*- coding: utf-8 -*-
import sys
import rospy
import time
import math
from math import sqrt
from pyzbar import pyzbar
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
from collections import defaultdict
import os 

rospy.init_node('computer_vision_sample')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

file = open('Report.txt', 'w+')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image)
image_pub1 = rospy.Publisher('~debugarrow', Image)
flag = False
def image():
    global xa
    global ya
    navigate_wait(x=xa, y=ya, z=0.6, yaw=0.0,speed=0.2, frame_id='aruco_map')
    for i in range(50):
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        hsv_min = np.array((81, 102, 39), np.uint8)
        hsv_max = np.array((130, 255, 186), np.uint8)
        height, width = img.shape[:2] #Находим размер изображения
            # преобразуем RGB картинку в HSV модель
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # применяем цветовой фильтр
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
                # вычисляем моменты изображения
        moments = cv2.moments(thresh, 1)
        dM01 = moments['m01']
        dM10 = moments['m10']
        dArea = moments['m00']
        image_pub1.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))
        # будем реагировать только на те моменты,
        # которые содержать больше 100 пикселей
        if dArea > 100:
            print('0.75 blue')
            return cv2.imread('blue.jpeg',0), 1.35
    navigate_wait(x=xa, y=ya, z=0.6, yaw=0.0,speed=0.2, frame_id='aruco_map')
    for i in range(50):
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        hsv_min = np.array((18, 139, 98), np.uint8)
        hsv_max = np.array((51, 255, 255), np.uint8)
        height, width = img.shape[:2] #Находим размер изображения
            # преобразуем RGB картинку в HSV модель
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # применяем цветовой фильтр
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
                # вычисляем моменты изображения
        moments = cv2.moments(thresh, 1)
        dM01 = moments['m01']
        dM10 = moments['m10']
        dArea = moments['m00']
        image_pub1.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))
        # будем реагировать только на те моменты,
        # которые содержать больше 100 пикселей
        if dArea > 100:
            print('0.25 yellow')
            return cv2.imread('yellow.jpeg',0), 0.85
    navigate_wait(x=xa, y=ya, z=0.6, yaw=0.0,speed=0.2, frame_id='aruco_map')
    for i in range(50):
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        hsv_min = np.array((0, 106, 172), np.uint8)
        hsv_max = np.array((12, 216, 255), np.uint8)
        height, width = img.shape[:2] #Находим размер изображения
            # преобразуем RGB картинку в HSV модель
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # применяем цветовой фильтр
        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
                # вычисляем моменты изображения
        moments = cv2.moments(thresh, 1)
        dM01 = moments['m01']
        dM10 = moments['m10']
        dArea = moments['m00']
        image_pub1.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))
        # будем реагировать только на те моменты,
        # которые содержать больше 100 пикселей
        if dArea > 100:
            print('1 red')
            return cv2.imread('red.jpeg',0),1.6
    print('0.5 black')
    return cv2.imread('black.jpeg',0), 1.1

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
        
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=True, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    global file
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        t=telem
        file.write('x={}, y={}, z={}, yaw={}\n'.format(t.x, t.y, t.z, t.yaw))
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
    	    break
        rospy.sleep(0.2)

start_time = time.time()
navigate_wait(z=1, frame_id='body', auto_arm=True)
#os.system('python telem.py')
navigate_wait(x=0.4, y=0.8, z=0.6, speed=0.2, frame_id='aruco_map')

#QR
while not flag:
    cv_image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    if not flag:
        for barcode in barcodes:
            b_data = barcode.data.encode("utf-8")
            b_type = barcode.type
            (x, y, w, h) = barcode.rect
            xc = x + w/2
            yc = y + h/2
            #print("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))
	    #print(b_data)
            if type(b_data) != type(None):
                flag = b_data
                

#print(flag)
data = flag.split()
N = data[-1]
xa, ya = data[-3], data[-2]
xa=float(xa)
ya=float(ya)
print(data)
#QR KON

#Облёт препятствий
g_graph={}

def get_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def prinad(x, y, x1, y1, x2, y2):
    if x1 < x < x2 and y1 < y < y2 and x2 >= x1 and y2 >= y1:
        return 1
    elif x1 < x < x2 and y1 > y > y2 and x2 >= x1 and y2 <= y1:
        return 1
    elif x1 > x > x2 and y1 < y < y2 and x2 <= x1 and y2 >= y1:
        return 1
    elif x1 > x > x2 and y1 > y > y2 and x2 <= x1 and y2 <= y1:
        return 1
    elif x1 == x2 and x1 == x and y1 > y > y2 and x2 == x1 and y2 <= y1:
        return 1
    elif x1 == x2 and x1 == x and y1 < y < y2 and x2 == x1 and y2 >= y1:
        return 1
    elif x1 > x > x2 and y == y1 and y == y2 and x2 <= x1 and y2 == y1:
        return 1
    elif x1 < x < x2 and y == y1 and y == y2 and x2 >= x1 and y2 == y1:
        return 1
    else:
        return 0


def graph(matrix):
    global g_graph
    for i in range(len(matrix)):
        g_graph[i + 1] = []
        for j in range(len(matrix)):
            if matrix[i][j] < 1000000000000:
                g_graph[i + 1] = g_graph.get(i + 1, []) + [j + 1]


ver = []
otcl = sqrt(0.27)
s = []
n = (len(data)-3)/2
print(n)
a = []
A = []
a.append(0.4)
a.append(0.8)
s.append(a)
a = []
a.append(xa)
a.append(ya)
s.append(a)
a=[]
KK=0
i=0
while i<n*2:
    a=[]
    A=[]
    a.append(float(data[i]))
    a.append(float(data[i+1]))
    #print(a[0])
    A.append(a[0]-otcl)
    A.append(a[1]-otcl)
    A.append(a[0]+otcl)
    A.append(a[1]-otcl)
    A.append(a[0])
    A.append(a[1]+otcl)
    s.append(A)
    i+=2
    print"Columm area x=",a[0],", y=",a[1]
print"Navigation area x=",xa,", y=",ya
print"Order number:",N
n=n+2
#print(s)
start = s[0]
finish = s[1]
a1 = start[1] - finish[1]  # y1 - y2
b1 = finish[0] - start[0]  # x2 - x1
c1 = start[0] * finish[1] - start[1] * finish[0]  # x1 * y2 - x2 * y1;
ln = (n - 2) * 3 + 2
a = [[1000000000000] * ln for i in range(ln)]
s1 = [start]
s1.append(start)
points = []
points.append(s[0])
i = 2
while i < n:
    p = [s[i][0], s[i][1]]
    points.append(p)
    p = [s[i][2], s[i][3]]
    points.append(p)
    p = [s[i][4], s[i][5]]
    points.append(p)
    i += 1
points.append(s[1])
#print(points)
for i in range(ln):
    j = i + 1
    start = points[i]
    while j < ln:
        finish = points[j]
        k = 2
        a1 = start[1]-finish[1]
        b1 = finish[0]-start[0]
        c1 = start[0] * finish[1] - start[1] * finish[0]
        kal = 0
        while k < n:
            tre = s[k]
            a2 = tre[1] - tre[3]
            b2 = tre[2] - tre[0]
            c2 = tre[0] * tre[3]-tre[2]*tre[1]
            det = a1 * b2 - a2 * b1
            if not det == 0:
                x = (b1 * c2 - b2 * c1) / det
                y = (a2 * c1 - a1 * c2) / det
                if prinad(x, y, tre[0], tre[1], tre[2], tre[3]) == 1:
                    if prinad(x, y, start[0], start[1], finish[0], finish[1]) == 1:
                        kal = 1000000000000
                        break
            a2 = tre[1]-tre[5]
            b2 = tre[4]-tre[0]
            c2 = tre[0]*tre[5]-tre[4]*tre[1]
            det = a1 * b2 - a2 * b1
            if not det == 0:
                x = (b1 * c2 - b2 * c1) / det
                y = (a2 * c1 - a1 * c2) / det
                if prinad(x, y, tre[0], tre[1], tre[4], tre[5])==1:
                    if prinad(x, y, start[0], start[1], finish[0], finish[1])==1:
                        kal = 1000000000000
                        break
            a2 = tre[3] - tre[5]
            b2 = tre[4] - tre[2]
            c2 = tre[2] * tre[5] - tre[4] * tre[3]
            det = a1 * b2 - a2 * b1
            if not det == 0:
                x = (b1 * c2 - b2 * c1) / det
                y = (a2 * c1 - a1 * c2) / det
                if prinad(x, y, tre[2], tre[3], tre[4], tre[5])==1:
                    if prinad(x, y, start[0], start[1], finish[0], finish[1])==1:
                        kal = 1000000000000
                        break
            k += 1

        if kal == 0:
            kal = (get_distance(start[0], start[1], finish[0], finish[1]))
        a[i][j] = kal
        a[j][i] = kal
        j += 1
#pprint(a)
#all_paths(ver, 0, ln-1)
graph(a)
#print(g_graph)


class Graph:
    def __init__(self, vertices):
        # Нет. вершин
        self.V = vertices

        # словарь по умолчанию для хранения графа
        self.graph = defaultdict(list)
        self.rides = []

    # функция добавления ребра в граф
    def addEdge(self, u, v):
        self.graph[u].append(v)

    '''Рекурсивная функция для печати всех путей от 'u' до 'd'.
    visit [] отслеживает вершины в текущем пути.
    path [] хранит актуальные вершины, а path_index является текущим
    индексом в path[]'''

    def printAllPathsUtil(self, u, d, visited, path):

        # Пометить текущий узел как посещенный и сохранить в path
        visited[list(self.graph.keys()).index(u)] = True
        path.append(u)

        # Если текущая вершина совпадает с точкой назначения, то
        # print(current path[])
        if u == d:
            self.rides.append(path[:])
        else:
            # Если текущая вершина не является пунктом назначения
            # Повторить для всех вершин, смежных с этой вершиной
            for i in self.graph[u]:
                if visited[list(self.graph.keys()).index(i)] == False:
                    self.printAllPathsUtil(i, d, visited, path)

        # Удалить текущую вершину из path[] и пометить ее как непосещенную
        path.pop()
        visited[list(self.graph.keys()).index(u)] = False

    # Печатает все пути от 's' до 'd'
    def printAllPaths(self, s, d):

        # Отметить все вершины как не посещенные
        visited = [False] * (self.V)

        # Создать массив для хранения путей
        path = []

        # Рекурсивный вызов вспомогательной функции печати всех путей
        self.printAllPathsUtil(s, d, visited, path)


g = Graph(len(g_graph.keys()))
for i, v in g_graph.items():
    for e in v:
        g.addEdge(i, e)


def find_length(ride):
    global a
    s = 0
    for i in range(len(ride) - 1):
        s += a[i][i + 1]
        # print(s, a[i][i + 1])
    # print()
    return ride, s



s = 1
d = ln
#print("".format(s, d))
g.printAllPaths(s, d)
#print(g.rides)
List_rides_with_length = list(map(find_length, g.rides))
print(min(List_rides_with_length, key=lambda s: s[1])[0])
kek=(min(List_rides_with_length, key=lambda s: s[1])[0])
for i in (min(List_rides_with_length, key=lambda s: s[1])[0]):
    print(i)
    navigate_wait(x=points[i-1][0], y=points[i-1][1], z=1,speed=0.2, frame_id='aruco_map')
    print"Flight x=",points[i-1][0]," y=",points[i-1][1]
kek.reverse()

def image_callback(data):
    global img
    img = bridge.imgmsg_to_cv2(data, 'bgr8')    
def stel():
    global N
    global sect
    global kek
    global points
    while True:
        hsv_min = np.array((60, 171, 58), np.uint8)
        hsv_max = np.array((88, 255, 146), np.uint8)
        navigate_wait(x=0.2, y=0, z=0, frame_id='body')
        for i in range(0,100):
            img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
            height, width = img.shape[:2] #Находим размер изображения
            # преобразуем RGB картинку в HSV модель
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # применяем цветовой фильтр
            thresh = cv2.inRange(hsv, hsv_min, hsv_max)
	    image_pub.publish(bridge.cv2_to_imgmsg(thresh, 'mono8'))
            # вычисляем моменты изображения
            moments = cv2.moments(thresh, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            # будем реагировать только на те моменты,
            # которые содержать больше 100 пикселей
            if dArea > 100:
                print('find')
                x = int(dM10 / dArea)
                y = int(dM01 / dArea)
                if height/2-50<y and y<height/2+50 and width/2-50<x and x<width/2+50:
                    navigate_wait(x=0.2, y=0, z=0, frame_id='body')
		    print('land')
                    land_wait()
                    rospy.sleep(1)
		    print('go back')
                    navigate_wait(x=0, y=0, z=0.5, frame_id='body')
                    for i in kek:
                        print(i)
                        navigate_wait(x=points[i-1][0], y=points[i-1][1], z=1,speed=0.2, frame_id='aruco_map')
                        print"Flight x=",points[i-1][0]," y=",points[i-1][1]
                    navigate_wait(x=0, y=0, z=1.0, frame_id='aruco_map')
                    land_wait()
                    print(N+' delivered in '+sect+' for '+"%s seconds" % (time.time() - start_time)%60+"%s seconds" % int((time.time() - start_time)/60))
                    sys.exit()
while True:
    navigate_wait(x=xa, y=ya, z=0.6, yaw=0.0,speed=0.2, frame_id='aruco_map')
    rospy.sleep(1)
    template,za=image()
    w, h = template.shape[::-1]
    for i in range(0,100):
	img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	print(max_val)
    	if max_val > 0.6:
            top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] +h)
            cv2.rectangle(img,top_left, bottom_right, 255, 2)
            image_pub1.publish(bridge.cv2_to_imgmsg(img, 'mono8'))
            print("180, Sector C required")
            sect='Sector C'
	    navigate_wait(x=xa, y=ya, z=za, yaw=0.0, frame_id='aruco_map')
            stel()
    navigate_wait(x=xa, y=ya, z=0.6, yaw=1.57, frame_id='aruco_map')
    rospy.sleep(1)
    for i in range(0,100):
	img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
	print(max_val)
        if max_val > 0.6:
            top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] +h)
            cv2.rectangle(img,top_left, bottom_right, 255, 2)
            image_pub1.publish(bridge.cv2_to_imgmsg(img, 'mono8'))
	    print("90, Sector B required")
	    sect='Sector B'
	    navigate_wait(x=xa, y=ya, z=za, yaw=1.57, frame_id='aruco_map')
            stel()
    navigate_wait(x=xa, y=ya, z=0.6, yaw=3.14, frame_id='aruco_map')
    rospy.sleep(1)
    for i in range(0,100):
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        print(max_val)
        if max_val > 0.6:
            top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] +h)
            cv2.rectangle(img,top_left, bottom_right, 255, 2)
            image_pub1.publish(bridge.cv2_to_imgmsg(img, 'mono8'))
            print("0, Sector D required")
            sect='Sector D'
    	    navigate_wait(x=xa, y=ya, z=za, yaw=3.14, frame_id='aruco_map')
            stel()
    navigate_wait(x=xa, y=ya, z=0.6, yaw=4.71, frame_id='aruco_map')
    rospy.sleep(1)
    for i in range(0,100):
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        print(max_val)
        if max_val > 0.6:
            top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] +h)
            cv2.rectangle(img,top_left, bottom_right, 255, 2)
            image_pub1.publish(bridge.cv2_to_imgmsg(img, 'mono8'))
            print("270, Sector A required")
            sect='Sector A'
            navigate_wait(x=xa, y=ya, z=za, yaw=4.71, frame_id='aruco_map')
	    stel()
   
