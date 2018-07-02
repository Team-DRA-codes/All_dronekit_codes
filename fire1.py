from firebase import firebase
firebase = firebase.FirebaseApplication("https://teamdra-41045.firebaseio.com", None)
result = firebase.get('/message', None)
a = str(result)
lat=float(a[11:21])
lon=float(a[34:44])
print(lat)
print(lon)
