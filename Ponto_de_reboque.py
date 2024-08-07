import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
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
h = 150 #Altura do Cg da carga [mm]
R = 266 #Raio da roda [mm]
M = 235 #Massa do carro [Kg]
g = -9.8 #Aceleração da gravidade [m/s^2]
S = 1383.65 #Distancia entre eixos
T_Motor = 20 * 1000 #Torque maximo do motor [N.mm]
Red_Tor_Traseira = 4*9.5 #Redução da cvt e da caixa
Red_Tot_Dianteira = 4*9.5 #Redução da cvt, da caixa e do diferencial
Coef_Atrito_R = 0.6 #Coeficiente de atrito das rodas com a superficie
Coef_Atrito_L = 0.8 #Coeficiente de atrito da carga com a superficie



#Parametros relacionados à analise:
X_min = -200 #Limite inferior de analise em "x" [mm]
X_max = 100 #Limite superior de analise em "x" [mm]
Y_min = 350 #Limite inferior de analise em "y" [mm]
Y_max = 950 #Limite superior de analise em "y" [mm]
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
MainCanvas = cv2.circle(MainCanvas, [Roda_x_pix,Roda_y_pix], radius=10, color=(255, 0, 255), thickness=-1) #As coordenadas para o circulo são (x,y) mesmo
#Coordenadas do C.G. do carro: (x,y) na nova imagem:
Cg_x_pix = Roda_x_pix + int(Cg_x/K_Corr)
Cg_y_pix = Roda_y_pix - int(Cg_y/K_Corr)
MainCanvas = cv2.circle(MainCanvas, [Cg_x_pix,Cg_y_pix], radius=10, color=(0, 255, 0), thickness=-1) #As coordenadas para o circulo são (x,y) mesmo
TemporaryCanvas = 255 * np.ones([Alt_Canv,Lrg_Canv,3], np.uint8)


frame_number = 1 
for x in range(X_min,X_max,passo):
    for y in range(Y_min,Y_max,passo):
        #Resetando a imagem temporaria:
        TemporaryCanvas[0:Alt_Canv , 0:Lrg_Canv] = MainCanvas
        #Convertemos essas coordenadas para o pixel mais proximo:
        y_pix = Roda_y_pix - int(y/K_Corr)
        x_pix = Roda_x_pix + int(x/K_Corr)



        #Calculamos a posição do C.G. da carga em relação ao centro da roda.
        Load_Cg_y = h-R #[mm]
        Load_Cg_x = x - np.sqrt(L**2 - ((y-Load_Cg_y)**2)) #[mm]
        Load_Cg_y_pix = Roda_y_pix - int(Load_Cg_y/K_Corr) #[pixel]
        Load_Cg_x_pix = MainCanvas.shape[1]-MainImage.shape[1]+Im_x_R+int(Load_Cg_x/K_Corr) #[pixel]

        #                   A N A L I S E   D E   F O R C A S 
        #Analise do levantamento da roda dianteira:
        #A condição da tensão máxima sobre a corda é a equivalencia do torque.
        # T = r x F = |r|. F|.sin(a)
        # a = arccos(r/|r| * F/|F|)
        #Analise de torque:
        Coord_Trq = np.array([0,0]) #Coordenadas do ponto de analise dos torques
        #Torque gerado pelo peso do carro:
        Coord_Cg = np.array([Cg_x,Cg_y])
        r = Coord_Trq - Coord_Cg #r [mm,mm]
        norm_r = np.linalg.norm(r) #|r| [mm]
        Fp = np.array([0,M*g]) #F [N,N]
        norm_Fp = np.linalg.norm(Fp) #|F| [N]
        a = np.arccos(np.dot( r/norm_r , Fp/norm_Fp) ) #Angulo entre o vetor que liga o C.G. ao ponto de contato e o vetor forca peso
        Torq_Cg = (norm_r * norm_Fp * np.sin(a)) #[N*m]
        #Torque gerado pelo motor na roda taseira:
        Torq_Mtr_Traseira = T_Motor * Red_Tor_Traseira # [N.mm]
        #Analise dos vetores no ponto de analise:
        Coord_xy = np.array([x,y]) #Ponto de analise
        Coord_Load_Cg = np.array([Load_Cg_x,Load_Cg_y])
        j = Coord_Trq - Coord_xy  #j (mm,mm)
        norm_j = np.linalg.norm(j) #|j| [mm]
        Lin_Ft = Coord_Load_Cg - Coord_xy #Vetor que define a linha de ação da tensão da corda
        norm_Lin_Ft = np.linalg.norm(Lin_Ft)
        b = np.arccos(np.dot(j/norm_j , Lin_Ft/norm_Lin_Ft )) #Angulo entre o vetor entre o ponto xy e ponto de contato com a linha de ação da tensão
        d = np.arccos(np.dot(np.array([1,0]) , Lin_Ft/norm_Lin_Ft ))


        #Condição de elevação da roda dianteira:
        F_Max_Elv = (Torq_Cg-Torq_Mtr_Traseira)/(norm_j*np.sin(b)) #Força de tensão máxima para a condição de elevação da roda dianteira [N]
        F_Hor_Max_Elv = F_Max_Elv * ( np.dot( Lin_Ft/norm_Lin_Ft , np.array([1,0]) ) )
        F_Hor_Max_Elv = abs(float("%.2f" %F_Hor_Max_Elv))
        F_Ver_Max_Elv = F_Max_Elv * ( np.dot( Lin_Ft/norm_Lin_Ft , np.array([0,1]) ) )
        F_Ver_Max_Elv = abs(float("%.2f" %F_Ver_Max_Elv))
        #Condição de limite força motriz 4x2:
        F_Hor_Max_Mot_4x2 = (-1)*Torq_Mtr_Traseira / R #Pelo equilibrio horizontal Fh = Fat, Fat é limitada pelo motor [N*mm / mm]
        F_Hor_Max_Mot_4x2 = abs(float("%.2f" %F_Hor_Max_Mot_4x2))
        F_Ver_Max_Mot_4x2 = F_Hor_Max_Mot_4x2 * np.tan(d)
        F_Ver_Max_Mot_4x2 = abs(float("%.2f" %F_Ver_Max_Mot_4x2))
        #Condição de escorregamento das rodas 4x2:
        F_Max_Esc_4x2 = (M*g*(Cg_x-S))/(norm_j*np.sin(b) + np.cos(d)/Coef_Atrito_R * (R*Coef_Atrito_R - S) + np.sin(d)*S)
        F_Hor_Max_Esc_4x2 = F_Max_Esc_4x2 * ( np.dot( Lin_Ft/norm_Lin_Ft , np.array([1,0]) ) )
        F_Hor_Max_Esc_4x2 = abs(float("%.2f" %F_Hor_Max_Esc_4x2))
        F_Ver_Max_Esc_4x2 = F_Max_Esc_4x2 * ( np.dot( Lin_Ft/norm_Lin_Ft , np.array([0,1]) ) )
        F_Ver_Max_Esc_4x2 = abs(float("%.2f" %F_Ver_Max_Esc_4x2))
        
        #Condição de escorregamento das rodas 4x4:
        F_Max_Esc_4x4 = (M*g*(Cg_x-S-R*Coef_Atrito_R))/(norm_j*np.sin(b) + np.cos(d)/Coef_Atrito_R * ((-1)*S) + np.sin(d)*S)
        F_Hor_Max_Esc_4x4 = F_Max_Esc_4x4 * ( np.dot( Lin_Ft/norm_Lin_Ft , np.array([1,0]) ) )
        F_Hor_Max_Esc_4x4 = abs(float("%.2f" %F_Hor_Max_Esc_4x4))
        F_Ver_Max_Esc_4x4 = F_Max_Esc_4x4 * ( np.dot( Lin_Ft/norm_Lin_Ft , np.array([0,1]) ) )
        F_Ver_Max_Esc_4x4 = abs(float("%.2f" %F_Ver_Max_Esc_4x4))
        #Analise e comparação dos resultados de força horizontal máxima:
        Lista_Forcas_Hor = [(F_Hor_Max_Elv) , (F_Hor_Max_Mot_4x2) , (F_Hor_Max_Esc_4x2) , (F_Hor_Max_Esc_4x4)]
        Lista_Forcas_Hor_Sorted = sorted(Lista_Forcas_Hor)
        Lista_Condicoes = ["Elevacao da dianteira" , "Falta de torque" , "Escorregamento 4x2" , "Escorregamento 4x4"]
        Lista_Condicoes_Sorted = [x for y, x in sorted(zip(Lista_Forcas_Hor, Lista_Condicoes))]
        Lista_Forcas_Ver = [(F_Ver_Max_Elv) , (F_Ver_Max_Mot_4x2) , (F_Ver_Max_Esc_4x2) , (F_Ver_Max_Esc_4x4)]
        Lista_Forcas_Ver_Sorted = [j for k ,j in sorted(zip(Lista_Forcas_Hor, Lista_Forcas_Ver))]
    



        
        #Desenha um ponto na coordenada de analise:
        TemporaryCanvas = cv2.circle(TemporaryCanvas, [x_pix,y_pix], radius=3, color=(0, 0, 0), thickness=2)
        #Desenha um ponto no C.G. da carga:
        TemporaryCanvas = cv2.circle(TemporaryCanvas, [Load_Cg_x_pix,Load_Cg_y_pix], radius=3, color=(0, 0, 255), thickness=2)
        #Desenha uma linha representando a corda:
        TemporaryCanvas = cv2.line(TemporaryCanvas, [x_pix,y_pix], [Load_Cg_x_pix,Load_Cg_y_pix], color=(0, 0, 0), thickness=2)
        #Desenha uma caixa para representar a carga:
        Lado_2 = int(h/K_Corr)-15
        start_point = (Load_Cg_x_pix-Lado_2,Load_Cg_y_pix-Lado_2) 
        end_point = (Load_Cg_x_pix+Lado_2,Load_Cg_y_pix+Lado_2) 
        TemporaryCanvas = cv2.rectangle(TemporaryCanvas, start_point, end_point, color=(0, 0, 255), thickness=5)
        #Escreve o Angulo b:
        Angulo = str(("%.2f" %(b*360/(2*3.1514))))
        Coord_Txt_angulo = np.array([x_pix-80,y_pix])
        TemporaryCanvas = cv2.putText(TemporaryCanvas , Angulo, Coord_Txt_angulo , cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,0,255),thickness=2,lineType=2)
        #Escreve a carga maxima em Kg:
        Coord_Txt_Força = np.array([Load_Cg_x_pix-40,Load_Cg_y_pix-Lado_2-10])
        Força_Horizontal_Maxima_Text = abs((Lista_Forcas_Hor_Sorted[0] + (Lista_Forcas_Hor_Sorted[0] * Coef_Atrito_L))/(g*Coef_Atrito_L))
        Força_Horizontal_Maxima_Text = str("%.1f" %Força_Horizontal_Maxima_Text +"Kg")
        TemporaryCanvas = cv2.putText(TemporaryCanvas , Força_Horizontal_Maxima_Text, Coord_Txt_Força , cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,0,255),thickness=2,lineType=2)
        #Escreve a condição de falha:
        Coord_Txt_Cond = np.array([50,100])
        for n in range(0,len(Lista_Condicoes_Sorted)):
            Coord_Txt_Cond = Coord_Txt_Cond + np.array([0,40])
            Cond_text = str(Lista_Condicoes_Sorted[n]) + ": " + str(Lista_Forcas_Hor_Sorted[n]) + "N;"
            TemporaryCanvas = cv2.putText(TemporaryCanvas , Cond_text, Coord_Txt_Cond, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(255,0,255),thickness=2,lineType=2)
        
        cv2.imwrite("Imagens\\"+str(frame_number)+".png", TemporaryCanvas)
        frame_number = frame_number + 1
        

# cv2.imshow("MainImage",MainCanvas)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
