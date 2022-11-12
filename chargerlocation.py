import json
import time

global counter
global GeoList
counter = 0
GeoList = list()
start = time.time()
while time.time() - start <40:
    try:
        
        

        with open('/home/gilblankenship/Projects/PythonCode/env/main/Charge/longdistance/location.json') as json_file:
            locationdict = json.load(json_file)
        json_file.close()
        x = locationdict['latitude']
        #print(x)
        y = locationdict['longetude']

        GeoList.insert(counter, (x,y))
        
        counter = counter + 1
    except json.decoder.JSONDecodeError as err:
        print(err)
        print("in json")
        #this breaks out of the function
time.sleep(3)
xx = 0
yy = 0

for i in range(len(GeoList)):
    x,y = GeoList(i)
    xx = xx+x
    yy = yy+y
xave = xx/range(len(GeoList)) #if this counts from zero one will need to be added as that wouldent be an average.
yave = yy/range(len(GeoList))

foruse ={"latitude": xave, "longetude": yave}
with open('chargerposition.json','w') as outfile:
    json.dump(foruse, outfile)
    #print("updated pos")
outfile.close()