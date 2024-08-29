import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from scipy.interpolate import griddata
import cv2

#Importação e tratamento inicial da imagem do carro:
MainImage = cv2.imread("J13.png")
Im_x_R = 132 #Coordenadas do centro da roda traseira [pixel]
Im_y_R = 561 #Coordenadas do centro da roda traseira [pixel]
EntreEixos_Pixel = 570 #Distancia eixos [pixel]
EntreEixos_mm = 1383.65 #Distancia entre eixos [mm]
K_Corr = int(EntreEixos_mm/EntreEixos_Pixel) #Coeficiente de correção [mm/pixel]



#definição de parametros relacionados ao projeto:
Cg_x = 500 #Coordenadas do centro de gravidade em relação ao centro da roda traseira [mm]
Cg_y = 250 #Coordenadas do centro de gravidade em relação ao centro da roda traseira [mm]
L = 1500 #Comprimento da corda [mm]
h = 200 #Altura do Cg da carga [mm]
R = 266 #Raio da roda [mm]
M = 235 #Massa do carro [Kg]
g = 9.8 #Aceleração da gravidade [m/s^2]
S = 1383.65 #Distancia entre eixos
T_Motor = 20 * 1000 #Torque maximo do motor [N.mm]
Red_Tor_Traseira = 4*9.5 #Redução da cvt e da caixa
Red_Tot_Dianteira = 4*9.5 #Redução da cvt, da caixa e do diferencial
Coef_Atrito_R = 0.6 #Coeficiente de atrito das rodas com a superficie
Coef_Atrito_L = 0.8 #Coeficiente de atrito da carga com a superficie



#Parametros relacionados à analise:
X_min = -300 #Limite inferior de analise em "x" [mm]
X_max = 300 #Limite superior de analise em "x" [mm]
Y_min = 0 #Limite inferior de analise em "y" [mm]
Y_max = 900 #Limite superior de analise em "y" [mm]
passo = 5



#Manipulações iniciais das imagens:
Alt_Main = MainImage.shape[0] #Altura da imagem do carro
Lrg_Main = MainImage.shape[1] #Largura da imagem do carro
Alt_Canv = Alt_Main #Altura na nova imagem
Lrg_Canv = Lrg_Main+int(L/K_Corr) #Largura da nova imagem
MainCanvas = 255 * np.ones([Alt_Canv,Lrg_Canv,3], np.uint8)
MainCanvas[0:Alt_Main,int(L/K_Corr):Lrg_Main+int(L/K_Corr)] = MainImage





#desenhando marcações nos pontos chave:
#Coordenadas do centro da roda: (x,y) na nova imagem:
Roda_x_pix = Lrg_Canv - Lrg_Main + Im_x_R
Roda_y_pix = Alt_Canv - Alt_Main + Im_y_R
# MainCanvas = cv2.circle(MainCanvas, [Roda_x_pix,Roda_y_pix], radius=10, color=(255, 0, 255), thickness=-1) #As coordenadas para o circulo são (x,y) mesmo
# Coordenadas do C.G. do carro: (x,y) na nova imagem:
Cg_x_pix = Roda_x_pix + int(Cg_x/K_Corr)
Cg_y_pix = Roda_y_pix - int(Cg_y/K_Corr)
# MainCanvas = cv2.circle(MainCanvas, [Cg_x_pix,Cg_y_pix], radius=10, color=(0, 255, 0), thickness=-1) #As coordenadas para o circulo são (x,y) mesmo
TemporaryCanvas = 255 * np.ones([Alt_Canv,Lrg_Canv,3], np.uint8)


#Criando as listas para armazenas os valores
List_Coord_x = []
List_Coord_y = []
List_Carga = []
List_F_Hor = []
List_Cor = []

List_F_Hor_4x4 = []

frame_number = 1 
for x in range(X_min,X_max,passo):
    for y in range(Y_min,Y_max,passo):
        



        # #                M A N I P U L A C A O   D A   I M A G E M                # 
        
        # #Resetando a imagem temporaria:
        # TemporaryCanvas[0:Alt_Canv , 0:Lrg_Canv] = MainCanvas
        
        # #Convertemos essas coordenadas para o pixel mais proximo:
        # y_pix = Roda_y_pix - int(y/K_Corr)
        # x_pix = Roda_x_pix - int(x/K_Corr)

        #Calculamos a posição do C.G. da carga em relação ao centro da roda.
        Load_Cg_y = h-R #[mm]
        Load_Cg_x = x - np.sqrt(L**2 - ((y-Load_Cg_y)**2)) #[mm]
        Load_Cg_y_pix = Roda_y_pix - int(Load_Cg_y/K_Corr) #[pixel]
        Load_Cg_x_pix = MainCanvas.shape[1]-MainImage.shape[1]+Im_x_R+int(Load_Cg_x/K_Corr) #[pixel]




        #                   A N A L I S E   D E   F O R C A S 
        #Analise do angulo Theta pelo ponto de analise:
        Coord_Trq = np.array([0,0]) #ponto de aplicação dos torques
        Coord_xy = np.array([x,y]) #Ponto de analise
        Coord_Load_Cg = np.array([Load_Cg_x,Load_Cg_y]) #Vetor de coordenadas do C.G. da carga.
        Lin_Ft = Coord_xy -  Coord_Load_Cg  #Vetor que define a linha de ação da tensão da corda
        norm_Lin_Ft = np.linalg.norm(Lin_Ft)
        theta = np.arccos(np.dot(np.array([1,0]) , Lin_Ft/norm_Lin_Ft )) #Angulo entre o vetor e a horizontal.

        #Condição 1: Elevação da roda dianteira:
        F_Hor_Max_Elv = (M*g*Cg_x)/(y + np.tan(theta)*x + R) #[N]
        F_Ver_Max_Elv = np.tan(theta) * (M*g*Cg_x)/(y + np.tan(theta)*x + R) #[N]
        Carga_Max_Elv = (F_Hor_Max_Elv/Coef_Atrito_L + F_Ver_Max_Elv)/g #[Kg]
        #print(F_Ver_Max_Elv)
        #print(np.tan(theta))

        #Condicao 2: Escorregamento dos pneus (4x4):
        F_Hor_Max_4x4 = Coef_Atrito_R * (M*g)/(1 - Coef_Atrito_R*np.tan(theta))
        F_Ver_Max_4x4 = np.tan(theta) * Coef_Atrito_R * (M*g)/(1 - Coef_Atrito_R*np.tan(theta))
        Carga_Max_4x4 = (F_Hor_Max_4x4/Coef_Atrito_L + F_Ver_Max_4x4)/g #[Kg]

        #Condicao 2: Escorregamento dos pneus (4x2):
        F_Hor_Max_4x2 = (M*g) * (Cg_x-S)/(y+np.tan(theta)*x + R + np.tan(theta)*S - S/Coef_Atrito_R)
        F_Ver_Max_4x2 = np.tan(theta) * (M*g) * (Cg_x-S)/(y+np.tan(theta)*x + R + np.tan(theta)*S - S/Coef_Atrito_R)
        Carga_Max_4x2 = (F_Hor_Max_4x2/Coef_Atrito_L + F_Ver_Max_4x2)/g #[Kg]

        #Analise e comparação dos resultados de força horizontal máxima:

        Lista_Carga = [(Carga_Max_Elv) , (Carga_Max_4x4)]
        Lista_Carga_Sorted = sorted(Lista_Carga)
        # Lista_Condicoes = ["Elevacao da dianteira"  , "Escorregamento 4x4" , "Escorregamento 4x2"]
        # Lista_Condicoes_Sorted = [x for y, x in sorted(zip(Lista_Carga, Lista_Condicoes))] #Criterio de comparação CARGA

        Lista_F_Hor = [F_Hor_Max_Elv,F_Hor_Max_4x4]
        Lista_F_Hor_Sorted = sorted(Lista_F_Hor)
        Lista_Condicoes = ["Elevacao da dianteira"  , "Escorregamento 4x4" , "Escorregamento 4x2"]
        Lista_Condicoes_Sorted = [x for y, x in sorted(zip(Lista_F_Hor, Lista_Condicoes))] #Criterio de comparação FORCA HORIZONTAL
        
        


        #                A R M A Z E N A N D O   V A L O R E S                  #
        Cor_Carga_Max_Elv = [1,0,0]
        Cor_Carga_Max_4x4 = [0,1,0]
        Cor_Carga_Max_4x2 = [0,0,1]
        Lista_Cores = [Cor_Carga_Max_Elv,Cor_Carga_Max_4x4]
        Lista_Cores_Sorted = [x for y, x in sorted(zip(Lista_Carga, Lista_Cores))]

        Carga_max = Lista_Carga_Sorted[0]
        Forca_Hor_Max = Lista_F_Hor_Sorted[0]
        Cor_Max = Lista_Cores_Sorted[0]

        List_Coord_x.append(x)
        List_Coord_y.append(y)
        List_Carga.append(Carga_max)
        List_F_Hor.append(Forca_Hor_Max)
        List_Cor.append(Cor_Max)

        List_F_Hor_4x4.append(F_Hor_Max_4x4)




        

        # #               M A N I P U L A C A O   D A   I M A G EM                #
        
        # #Desenha um ponto na coordenada de analise:
        # TemporaryCanvas = cv2.circle(TemporaryCanvas, [x_pix,y_pix], radius=3, color=(0, 0, 0), thickness=2)
        
        # #Desenha um ponto no C.G. da carga:
        # TemporaryCanvas = cv2.circle(TemporaryCanvas, [Load_Cg_x_pix,Load_Cg_y_pix], radius=3, color=(0, 0, 255), thickness=2)
        
        # #Desenha uma linha representando a corda:
        # TemporaryCanvas = cv2.line(TemporaryCanvas, [x_pix,y_pix], [Load_Cg_x_pix,Load_Cg_y_pix], color=(0, 0, 0), thickness=2)
        
        # #Desenha uma caixa para representar a carga:
        # Lado_2 = int(h/K_Corr)-15
        # start_point = (Load_Cg_x_pix-Lado_2,Load_Cg_y_pix-Lado_2) 
        # end_point = (Load_Cg_x_pix+Lado_2,Load_Cg_y_pix+Lado_2) 
        # TemporaryCanvas = cv2.rectangle(TemporaryCanvas, start_point, end_point, color=(0, 0, 255), thickness=5)
        
        # #Escreve o Angulo theta:
        # Angulo = str(("%.2f" %(theta*360/(2*3.1514))))
        # Coord_Txt_angulo = np.array([x_pix-80,y_pix])
        # TemporaryCanvas = cv2.putText(TemporaryCanvas , Angulo, Coord_Txt_angulo , cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,0,255),thickness=2,lineType=2)
        
        # #Escreve a condição de falha:
        # Titulo_falha = "Carga maxima [Kg]:"
        # Coord_Txt_Cond = np.array([50,100])
        # TemporaryCanvas = cv2.putText(TemporaryCanvas , Titulo_falha , Coord_Txt_Cond, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255,0,255),thickness=2,lineType=2)
        # for n in range(0,len(Lista_Condicoes_Sorted)):
        #     Coord_Txt_Cond = Coord_Txt_Cond + np.array([0,40])
        #     Cond_text = str("  -" + Lista_Condicoes_Sorted[n]) + ": " + str("%.2f" %Lista_Carga_Sorted[n]) + "Kg;"
        #     TemporaryCanvas = cv2.putText(TemporaryCanvas , Cond_text, Coord_Txt_Cond, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255,0,255),thickness=2,lineType=2)
        
        # #Salva a imagem:
        # cv2.imwrite("Imagens\\"+str(frame_number)+".png", TemporaryCanvas)
        # frame_number = frame_number + 1
        

# Plotar as listas de resultados
Background = cv2.cvtColor(MainCanvas, cv2.COLOR_BGR2RGB)  # Converte para RGB para exibir com Matplotlib

#Grafico da carga máxima
fig, ax = plt.subplots()
ax.imshow(Background, extent=[Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr,(-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr])  
cax = plt.scatter(List_Coord_x,List_Coord_y,edgecolors='none',s=1,c=List_Carga,cmap="inferno")
cbar = fig.colorbar(cax, ax=ax, orientation='vertical')
cbar.set_label("Carga Maxima [Kg]")
ax.set_xlim(Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr)
ax.set_ylim((-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr)
fig.savefig("Carga_Maxima_L="+ str(L) +"_h="+ str(h) +".png" , dpi=300)
#Grafico da força horizontal máxima
fig, bx = plt.subplots()
bx.imshow(Background, extent=[Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr,(-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr])  
cbx = plt.scatter(List_Coord_x,List_Coord_y,edgecolors='none',s=1,c=List_F_Hor,cmap="inferno")
cbar = fig.colorbar(cbx,ax=bx, orientation='vertical')
cbar.set_label("Força horizonal máxima [N]")
bx.set_xlim(Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr)
bx.set_ylim((-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr)
fig.savefig("For_Horizontal_L="+ str(L) +"_h="+ str(h) +".png", dpi=300)
#Grafico da condição de falha
fig, cx = plt.subplots()
cx.imshow(Background, extent=[Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr,(-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr])  
ccx = plt.scatter(List_Coord_x,List_Coord_y,edgecolors='none',s=1,c=List_Cor,cmap="inferno")
cbar = fig.colorbar(ccx,ax=cx, orientation='vertical')
cbar.set_label("Carga Maxima [Kg]")
cx.set_xlim(Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr)
cx.set_ylim((-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr)
fig.savefig("Cor_Max_L="+ str(L) +"_h="+ str(h) +".png", dpi=300)
#Grafico da condição de falha 4x4 especificamente
fig, dx = plt.subplots()
dx.imshow(Background, extent=[Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr,(-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr])  
cdx = plt.scatter(List_Coord_x,List_Coord_y,edgecolors='none',s=1,c=List_F_Hor_4x4 ,cmap="inferno")
cbar = fig.colorbar(cdx,ax=dx, orientation='vertical')
cbar.set_label("Força horizontal máxima 4x4 [N]")
dx.set_xlim(Roda_x_pix*K_Corr,(-1)*(Lrg_Canv-Roda_x_pix)*K_Corr)
dx.set_ylim((-1)*(Alt_Canv-Roda_y_pix)*K_Corr,Roda_y_pix*K_Corr)
fig.savefig("F_Hor_Max_4x4_L="+ str(L) +"_h="+ str(h) +".png", dpi=300)
plt.show()