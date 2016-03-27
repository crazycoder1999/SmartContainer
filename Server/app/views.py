from flask import render_template
from app import app
from flask.ext import shelve
from flask import request

#items = {};
#items['a1b2c3'] = item;

@app.route('/')
@app.route('/index')
def index():
	user = {'nickname': 'Miguel'}  # fake user
	db = shelve.get_shelve('c')
	items = db.values();
	return render_template('list.html',
				title='Home',
				user = user,
				items=items)

#Param: macAddres,position,threshold,label
#Called by the client application, in the configuration process of the container
#return 1 if the device is already registered
#return -1 if the parameter are missing
@app.route('/updateAll')
def updateAll():
	macAddr = reqParam(request,'macAddr');
	position = reqParam(request,'position');
	threshold = reqParam(request,'threshold');
	label = reqParam(request,'label');
	if macAddr == -1 or position == -1 or threshold == -1 or label == -1:
		return "-1";
	db = shelve.get_shelve('c');
	if not macAddr in db:
		print "ERROR MISSING DATA FOR: 	" + macAddr;
		return "-1";
	container = db[macAddr];
	container['position'] = position;
	container['threshold'] = threshold;
	container['label'] = label;	
	db[macAddr] = container;
	db.close();
	return "1"

#Param: mac address
#return 1 if the device is already registered
#return 0 if the device is not registered
#return -1 if the parameter are missing...
@app.route('/register')
def register():
	macAddr = reqParam(request,'macAddr');
	if macAddr == -1:
		return "-1";
	db = shelve.get_shelve('c')
	if macAddr in db:
		print "Container already registered: " + macAddr;
		return "1";
	else:
		print "New Container: Registering it";
		container = { 'macAddr':macAddr,"weight":0,"threshold":0,"position":"N/A","label":"empty"};
		db[macAddr] = container;
		db.close();
		return "0";

#Param: Wifi MacAddress
#return the label for the container
#return "-1" if the device is not registered
#TODO CHANGE TO READ ONLY!
@app.route('/getLabel')
def getLabel():
	macAddr = reqParam(request,'macAddr');
	if macAddr == -1:
		return "-1";
	db = shelve.get_shelve('c')
	lab = "-1";
	if macAddr in db:
		lab = db[macAddr]['label'];
	db.close();
	return lab;

#Param: Wifi MacAddress, position
#return "1" ALL OK
#return "-1" if the device is not registered or params are missing
@app.route('/setPosition')
def setPosition():
	macAddr = reqParam(request,'macAddr');
	position = reqParam(request,'position');
	return setContainerRequest(macAddr,'position',position);

#Param: Wifi MacAddress,label
#return "1" ALL OK
#return "-1" if the device is not registered or params are missing
@app.route('/setLabel')
def setLabel():
	macAddr = reqParam(request,'macAddr');
	label = reqParam(request,'label');
	return setContainerRequest(macAddr,'label',label);

#Param: macAddr, weight
#Update the weight of the device and send the alarm status back
#return "1" if alarm is ON: the weight is less than the threshold
#return "0" if alarm is OFF: the weight is more or equal the threshold value
#return "-1" if the device is not recognized
@app.route('/sendUpdate')
def sendUpdate():
	macAddr = reqParam(request,'macAddr');
	weight = reqParam(request,'weight');
	if macAddr == -1 or weight == -1:
		return "-1";
	if ( int(weight) < 0 ):
		weight = 0;
		print "Weight less than 0! The load cell is damaged!";
	db = shelve.get_shelve('c');
	if macAddr in db:
		devThreshold = db[macAddr]['threshold'];
		updateContainer(db,macAddr,'weight',weight);
		db.close();	
		if int(weight) >= int(devThreshold):#no alarm
			return "0";
		else:			  #alarm on
			return "1";
	db.close();
	return "-1";

#Param: macAddr, threshold
#Set the threshold value of the device
#return "1" if the new value is set
#return "-1" if the device is missing
@app.route('/setThreshold')
def setThreshold():
	macAddr = reqParam(request,'macAddr');
	threshold = reqParam(request,'threshold');
	return setContainerRequest(macAddr,'threshold',threshold);

#Param: macAddr
#Delete the container based on MacAddress
#return "1" if the new value is set
#return "-1" if the device is missing
@app.route('/delContainer')
def delContainer():
	macAddr = reqParam(request,'macAddr');
	if macAddr == -1:
		return "-1";
	db = shelve.get_shelve('c');
	del db[macAddr];
	db.close();
	return "1";

#utility functions!
def setContainerRequest(macAddr,param,paramValue):
	if macAddr == -1 or paramValue == -1:
		return "-1";
	db = shelve.get_shelve('c');
	res = updateContainer(db,macAddr,param,paramValue);
	db.close();
	return res;

def updateContainer(db,macAddr,param,value):
	if not macAddr in db:
		print "ERROR MISSING DATA FOR: 	" + macAddr;
		return "-1";
	container = db[macAddr];
	container[param] = value;
	db[macAddr] = container;
	return "1";
	
def reqParam(request,param):
	if param in request.args:
		return request.args[param];
	return -1;	

