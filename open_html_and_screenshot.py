#!/usr/bin/env python

#Prerequisite 
#(1) sudo apt-get install chromium-chromedriver
#(2) sudo pip install selenium

from selenium import webdriver
import time
import sys
import os

# maximized window
options = webdriver.ChromeOptions()
options.add_argument("--start-maximized")
# for chrome browser
driver = webdriver.Chrome('/usr/lib/chromium-browser/chromedriver', chrome_options = options)

#driver = webdriver.Firefox()

#print(sys.argv[1])
# driver.get(sys.argv[1])
path = 'file://'
path = path +  os.getcwd() + '/my_map.html'
print("html file path:{}".format(path))
# driver.get('file:///home/horizon_ad/siyuan/all_kinds_of_ros_pack/read_data_from_bag/my_map.html')
driver.get(path)
time.sleep(3)
driver.save_screenshot('map.png')
# driver.quit()
