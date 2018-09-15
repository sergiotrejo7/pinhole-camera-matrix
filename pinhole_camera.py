from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
from numpy.linalg import inv
from pylab import plot, show, figure, scatter, axes, draw
from matplotlib.widgets import Slider  # import the Slider widget
from os import system

fig = plt.figure()


#vectores de coordenadas del objeto
xi =[1 ,-1,-1,0 ,1 ,1 ,1 ,-1,-1,-1,-1,-1,-1,0 ,0 ,0 ,1 ,1 ,1 ,1]
yi =[-1,-1,1 ,2 ,1 ,-1,-1,-1,-1,-1,1 ,1 ,1 ,2 ,2 ,2 ,1 ,1,1,-1]
zi =[5,5,5,5,5,5,10,10,5,10,10,5,10,10,5,10,10,5,10,10]

###########  posicion de los slider en la ventana (X,Y,ancho X,alto Y)
f_slider = plt.axes([0.55, 0.1, 0.3, 0.02])
s_slider = plt.axes([0.55, 0.08, 0.3, 0.02])
mx_slider = plt.axes([0.55, 0.06, 0.3, 0.02])
my_slider = plt.axes([0.55, 0.04, 0.3, 0.02])
px_slider = plt.axes([0.55, 0.02, 0.3, 0.02])
py_slider = plt.axes([0.55, 0.0, 0.3, 0.02])

angx_slider = plt.axes([0.1, 0.04, 0.3, 0.02])
angy_slider = plt.axes([0.1, 0.02, 0.3, 0.02])
angz_slider = plt.axes([0.1, 0.0, 0.3, 0.02])
tx_slider = plt.axes([0.1, 0.1, 0.3, 0.02])
ty_slider = plt.axes([0.1, 0.08, 0.3, 0.02])
tz_slider = plt.axes([0.1, 0.06, 0.3, 0.02])
k_slider = plt.axes([0.1, 0.12, 0.3, 0.02])

k_slider = Slider(k_slider,      # the axes object containing the slider
                  'k',           # the name of the slider parameter
                  -10,           # minimal value of the parameter
                  10,            # maximal value of the parameter
                  valinit=1      # initial value of the parameter
                 )
s_slider = Slider(s_slider,      # the axes object containing the slider
                  's',            # the name of the slider parameter
                  -10,          # minimal value of the parameter
                  10,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter
                 )
f_slider = Slider(f_slider,      # the axes object containing the slider
                  'f',            # the name of the slider parameter
                  -30,          # minimal value of the parameter
                  30,          # maximal value of the parameter
                  valinit=1  # initial value of the parameter
                 )                 
angx_slider = Slider(angx_slider,      # the axes object containing the slider
                  'Giro x',            # the name of the slider parameter
                  -180,          # minimal value of the parameter
                  180,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter
                 )
angy_slider = Slider(angy_slider,      # the axes object containing the slider
                  'Giro y',            # the name of the slider parameter
                  -180,          # minimal value of the parameter
                  180,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter
                 )
angz_slider = Slider(angz_slider,      # the axes object containing the slider
                  'Giro z',            # the name of the slider parameter
                  -180,          # minimal value of the parameter
                  180,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter
                  )
tx_slider = Slider(tx_slider,      # the axes object containing the slider
                  'tx',            # the name of the slider parameter
                  -20,          # minimal value of the parameter
                  20,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter                  
                 )
ty_slider = Slider(ty_slider,      # the axes object containing the slider
                  'ty',            # the name of the slider parameter
                  -20,          # minimal value of the parameter
                  20,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter                  
                 )
tz_slider = Slider(tz_slider,      # the axes object containing the slider
                  'tz',            # the name of the slider parameter
                  -30,          # minimal value of the parameter
                  30,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter                  
                 )       
mx_slider = Slider(mx_slider,      # the axes object containing the slider
                  'mx',            # the name of the slider parameter
                  -20,          # minimal value of the parameter
                  20,          # maximal value of the parameter
                  valinit=1  # initial value of the parameter 
                  )
my_slider = Slider(my_slider,      # the axes object containing the slider
                  'my',            # the name of the slider parameter
                  -20,          # minimal value of the parameter
                  20,          # maximal value of the parameter
                  valinit=1  # initial value of the parameter    
                  )  
px_slider = Slider(px_slider,      # the axes object containing the slider
                  'px',            # the name of the slider parameter
                  -20,          # minimal value of the parameter
                  20,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter 
                  )
py_slider = Slider(py_slider,      # the axes object containing the slider
                  'py',            # the name of the slider parameter
                  -20,          # minimal value of the parameter
                  20,          # maximal value of the parameter
                  valinit=0  # initial value of the parameter    
                  )                                                                             
def update(val):
	#################### Matriz de camara K #####################
	# parametros interiores K
	km=k_slider.val				#factor de amplificacion
	f=f_slider.val				#distancia focal
	s=s_slider.val				#factor de distorsion
	mx=mx_slider.val			#factor en x
	my=my_slider.val			#factor en y
	px=px_slider.val			#posicion x del centro del sensor
	py=py_slider.val			#posicion y del centrl del sensor
	
	K=np.array([[  mx*f ,   s   , mx*px ],
				[   0   ,  my*f , my*py ],
				[   0   ,   0   ,   1   ]])
				
	################################# parametros exteriores R
	ang_x=angx_slider.val		#angulo con eje x
	ang_y=angy_slider.val		#angulo con eje y
	ang_z=angz_slider.val		#angulo con eje z
	thetax=np.deg2rad(ang_x)	#conversion de grados a radianes
	thetay=np.deg2rad(ang_y)
	thetaz=np.deg2rad(ang_z)
	#Matriz de rotacion en x
	Qx=np.array([[1,      0        ,       0       ], 
				[0 ,np.cos(thetax) ,-np.sin(thetax)], 
				[0 , np.sin(thetax), np.cos(thetax)]])
	#Matriz de rotacion en y			
	Qy=np.array([[np.cos(thetay) , 0 , np.sin(thetay)], 
				[        0       , 1 ,       0       ], 
				[-np.sin(thetay) , 0 , np.cos(thetay)]])
	#Matriz de rotacion en z			
	Qz=np.array([[np.cos(thetaz) ,-np.sin(thetaz), 0], 
				[np.sin(thetaz)  , np.cos(thetaz), 0], 
				[       0        ,       0       , 1]])
	
	t=np.array([[tx_slider.val,ty_slider.val,tz_slider.val]])			# Vector de traslacion t (1x3)
	R=np.matmul(np.matmul(Qx,Qy),Qz)									# Matriz de rotacion R (3x3)
	Rt=np.concatenate((R,t.T),axis=1)					#concatena la matriz R con el vector vertical t (3x(3+1))
	k=np.array([[km,0,0],[0,km,0],[0,0,1]])								# Matriz de amplificacion k (3x3)
	kRt=np.matmul(k,Rt)													# Matriz de kR|t (3x4)
###################### Creacion del mapeo X=Px  ################################
	X=np.array([xi,yi,zi,[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])	# matriz objeto (4x20)
	P=np.matmul(K,kRt)													# P=KR[I|t] (3x4)
	x=np.matmul(P,X)													# Vector x=PX (3x20)=(3x4)(4x20)
	xh=np.array([x[0]/x[2],x[1]/x[2],f*x[2]/x[2]])						# Vector x homogeneo (3x20)
	C=-np.matmul(inv(R),t.T) 									#Centro de camara C=-r^1(-T)*t (3x1)=(3x3)(3x1)
	x3D=np.matmul(kRt.T,xh)					#vector x de posicion del plano en el espacio 3D (4,20)=(4x3)(3x20)
	system('clear')
	print "Matriz X de puntos en espacio mundo:"
	print X
	print "\n Matriz K (parametros internos):"
	print K
	print "\n Matriz R de rotacion:"
	print R
	print "\n Vector t:"
	print t
	print "\n Matriz kR|t (parametros externos):"
	print kRt
	print "\n Matriz de camara P = KR|t:"
	print P
	print "\n Matriz x de puntos de plano imagen:"
	print x
	print "\n Matriz x del plano imagen en el espacio 3D:"
	print x3D
	print "\n Vector C"
	print C
	
	
	####### Para imprimir en Scatter 3D
	plt.ion()
	world = fig.add_subplot(1,2,1, projection='3d')						#crea area de impresion 3D
	#world.clear()
	world.plot_wireframe(X[0],X[1],X[2], rstride=10, cstride=10)			#imprime lineas del objeto
	world.plot_wireframe(x3D[0]+C[0],x3D[1]+C[1],x3D[2]+C[2], rstride=10, cstride=10)		#imprime lineas del plano imagen virtual
	world.plot_wireframe(-x3D[0]+C[0],-x3D[1]+C[1],-x3D[2]+C[2], rstride=10, cstride=10)	#imprime lineas en plano imagen real
	world.scatter(0,0,0, c='white', marker='o')						#imprime el centro de la camara(pinhole)
	world.scatter(C[0],C[1],C[2], c='black', marker='o')			#imprime posicion inicial del pinhole
	
	####### impresion de rotulos de los ejes
	world.set_xlabel('X Label')
	world.set_ylabel('Y Label')
	world.set_zlabel('Z Label')

	####### impresion del plano imagen
	world = fig.add_subplot(1,2,2)										#crea area de impresion 2D
	world.clear()		
	world.plot(-xh[0],-xh[1])											#imprime imagen en 2D
	
# actualizacion de los cambios de cada variable
k_slider.on_changed(update)
s_slider.on_changed(update)
f_slider.on_changed(update)
angx_slider.on_changed(update)
angy_slider.on_changed(update)
angz_slider.on_changed(update)
tx_slider.on_changed(update)
ty_slider.on_changed(update)
tz_slider.on_changed(update)
mx_slider.on_changed(update)
my_slider.on_changed(update)
px_slider.on_changed(update)
py_slider.on_changed(update)

plt.show()
