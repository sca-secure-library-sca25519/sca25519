from sage.misc.prandom import getrandbits
set_random_seed(0)

F = FiniteField(2^255-19)
a = F(486662)
x = F(9)
y = F(x^3 + a*x^2 + x).sqrt()
p = 2^255-19
E = EllipticCurve(GF(2^255-19),[0,486662,0,1,0])
B = E(x,y)
l = B.order()

def printKey(k):
    hexK = Integer(k).hex()
    # print(hexK)
    zeroK = (64 -  len(hexK)) * "0"
    #print("zeroes " + (64 - len(hexK)) * "0")
    hexK = zeroK + hexK

    #k_str = "{"
    k_str = ""
    for i in range(31):
        #k_str += "0x" + hexK[62-i*2:64-i*2] + ", "
        k_str += hexK[62-i*2:64-i*2] + " "
    #k_str += "0x" + hexK[62-31*2:64-31*2] + "}"
    k_str += hexK[62-31*2:64-31*2] + ""
    #print("k= " + k_str)
    print(k_str)

def printPoint(P, name):
    hexPx = Integer(P[0]).hex()
    hexPy = Integer(P[1]).hex()
    zeroX = (64 -  len(hexPx)) * "0"
    zeroY = (64 -  len(hexPy)) * "0"
    #print("zeroes " + (64 -  len(hexPx)) * "0")
    hexPx = zeroX + hexPx
    hexPy = zeroY + hexPy

    #Px = "{"
    #Py = "{"
    Px = ""
    Py = ""
    for i in range(31):
        #Px += "0x" + hexPx[62-i*2:64-i*2] + ", " 
        #Py += "0x" + hexPy[62-i*2:64-i*2] + ", "
        Px += hexPx[62-i*2:64-i*2] + " " 
        Py += hexPy[62-i*2:64-i*2] + " "
    #Px += "0x" + hexPx[62-31*2:64-31*2] + "}"
    #Py += "0x" + hexPy[62-31*2:64-31*2] + "}"
    Px += hexPx[62-31*2:64-31*2] + ""
    Py += hexPy[62-31*2:64-31*2] + ""
    #print(name + "x= " + Px)
    #print(name + "y= " + Py)
    #if (len(zeroX) > 0):
    #	print("lenX " + str(64 - len(zeroX)))
    print(Px)
    #if (len(zeroY) > 0):
    #	print("lenY " + str(64 - len(zeroY)))
    print(Py)

def genPoints():
    # Generate key
    k = getrandbits(256)

    # Clamp key
    k = k & 0xFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF8
    k = k & 0x7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
    k1 = k | 0x4000000000000000000000000000000000000000000000000000000000000000

    k2 = k1 >> 3

    #printKey(k1)
    printKey(k2)

    # Generate random R
    s = getrandbits(100)
    R = B*s
    printPoint(R, 'R')
    S = R*k1
    printPoint(S, 'S')
    return [k2, B, S]

for i in range(10000):
	genPoints()
	print()
# genPoints()
