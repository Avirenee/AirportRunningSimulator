## Importations
import random as rd
from math import floor
import string
import time
import matplotlib.pyplot as plt
import numpy as np
## Caracteristiques Orly
orly = [['c'],['h'],['a','d'],['f','c','e'],['g','d','m'],['d'],['e','l'],['b','i'],['h','j','k','l'],['i'],['i'],['i','g','m'],['e','l','n','s'],['m'],['C'],['q','s','r'],['p'],['p'],['m','t','p'],['s','u','z','E','B'],['v','t'],['u'],['z'],['t','w'],['C','B','D'],['t','A'],['o','A'],['A'],['t']]
distance_orly = [[1.4],[1.4],[1.4,4.5],[2.27,4.5,2.1],[2.47,2.1,2.74],[2.27],[2.47,2.44],[1.4,2.82],[2.82,1.27,2.82,2.23],[1.27],[2.82],[2.23,2.44,3.82],[2.74,3.82,3.39,3.32],[2.39],[2.52],[1.4,1.45,2.34],[1.4],[2.34],[3.32,4.97,1.45],[4.97,7.58,2.94,2.94,2.34],[2.35,7.58],[2.35],[4.42],[2.94,4.42],[4.10,3.01,2.34],[2.34,3.01],[2.52,4.10],[2.34],[2.94]]

xy_orly = [[1,2],[11,13],[2,1],[5.2,4.2],[6.6,5.6],[5,6.5],[6.5,8.1],[12,12],[10,10],[9.1,11],[7.3,9],[9,8],[9,4.2],[10,2],[28,8],[12.8,4.4],[13.4,3.1],[15,3.7],[12,5.6],[16.7,7.1],[17.6,14.6],[20,15],[18,0],[16.5,4.2],[21.9,8.3],[19,7.6],[25.9,9.4],[22,6],[18.6,4.9]]
dep_orly = [11,18,19]
arr_orly = [0,1,13,14,21,22]

## Caracteristiques Bordeaux
bordeaux=[[1],[0,2],[1,3,7,11],[2,4],[3,5],[4,6],[5],[2,8],[7,9,10],[8],[8],[2,12],[11,13],[12]]
xy_bordeaux=[[4,22],[18,20],[24,18],[30,15],[35,13],[39,11],[38,8],[22,15],[16,8],[4,4],[10.5,11.2],[27,21],[32,26],[31,34]]
distance_bordeaux=[[1600],[1600,292],[292,372,246,428],[372,282],[282,323],[323,213],[213],[246,590],[590,1070,683],[1070],[683],[428,308],[308,486],[486]]
#depart possible LDE=11.3.4
dep_bordeaux=[11,3,4]
#arrivée possible AGJN=0.6.9.13
arr_bordeaux=[0,6,9,13]
nb_porte = 6

def TransfoStrNbr(L):
    min = string.ascii_lowercase
    maj = string.ascii_uppercase
    StrAlph = min[:23]+'z'+maj
    ListeFinal = [[] for k in range(len(L))]
    for k in range(len(L)):
        for j in range(len(L[k])):
            if StrAlph.index(L[k][j]) > 28:
                print('L[k] = ', L[k])
                print('j =',j)
            ListeFinal[k].append(StrAlph.index(L[k][j]))
    return ListeFinal

def ProdCroix(n,distance):
    for k in range(len(distance)):
        for j in range(len(distance[k])):
            distance[k][j] = int(distance[k][j]*n)
    return distance

# L , xy , distance , dep , arr = bordeaux , xy_bordeaux , distance_bordeaux , dep_bordeaux , arr_bordeaux
L , xy , distance , dep , arr = TransfoStrNbr(orly) , xy_orly , ProdCroix(145,distance_orly) , dep_orly , arr_orly


## Programmes (Creation graphe, Ajout distances, Dijkstra)
inf = float (" inf")
def CreationGraphe(L):
    global graphe
    graphe=[[inf]*len(L) for k in range(len(L))]
    for i in range(len(graphe)):
        graphe[i][i]=0
        for j in range(len(L[i])):
            temp=L[i][j]
            graphe[i][temp]=distance[i][j]
    #print(graphe)
CreationGraphe(L)

def listeSommetsVoisins ( sommet ) :
    nbSommets = len ( graphe )
    ListeVoisins = []
    for k in range ( nbSommets ) :
        if k != sommet and graphe [ sommet ][k] > 0:
            ListeVoisins . append (k)
    return ( ListeVoisins )

def plusCourtChemin ( depart , cible ) :
    nbSommets = len ( graphe ) #nbSommets=14
    distancePar = [ inf for k in range ( nbSommets ) ] # 14 inf (distance parcourue)
    sommetPrecedent = [ -1 for k in range ( nbSommets ) ] #14 -1 (sommet precedent)
    nonParcourus = [k for k in range ( nbSommets ) ] # 14 non parcourus
    distancePar [ depart ] = 0
    nonParcourus_temp=[]
    temp=0
    n=0
    while len ( nonParcourus ) >0 :
        if len(nonParcourus)==temp:
            n=n+1
            if n>3:
                return False
        else:
            temp=len(nonParcourus)
        temp=len(nonParcourus)
        #print('NonParcourus=',nonParcourus)
        distMin = inf
        sommet1 = -1
        for sommet in nonParcourus : #parmi les sommets non parcourus
            if distancePar [ sommet ] < inf and ( distancePar [ sommet ] < distMin or
                distMin == inf) :
                distMin = distancePar [ sommet ]
                sommet1 = sommet
        if sommet1 in nonParcourus:
            nonParcourus . remove ( sommet1 )
        for sommet2 in listeSommetsVoisins ( sommet1 ) :
            distParS1 = distMin + graphe [ sommet1 ][ sommet2 ]
            if distancePar [ sommet2 ]== inf or distancePar [ sommet2 ] > distParS1 :
                distancePar [ sommet2 ] = distParS1
                sommetPrecedent [ sommet2 ] = sommet1
        #print ( distancePar , sommet1 )
    chemin = []
    sommet = cible
    while sommet > - 1:
        chemin =[ sommet ]+ chemin
        sommet = sommetPrecedent [ sommet ]
    return ( chemin )

def Graphe2(xy,L):
    tempx=[]
    tempy=[]
    x=[]
    y=[]
    for j in range(len(L)):
        if not len(tempx)==0:
            tempx=[]
            tempy=[]
        if len(L[j])==1:
            tempx.append(xy[j][0])
            tempx.append(xy[L[j][0]][0])
            tempy.append(xy[j][1])
            tempy.append(xy[L[j][0]][1])
            plt.plot(tempx,tempy,color='green')
        else:
            for k in range(len(L[j])):
                tempx.append(xy[j][0])
                tempx.append(xy[L[j][k]][0])
                tempy.append(xy[j][1])
                tempy.append(xy[L[j][k]][1])
            plt.plot(tempx,tempy,color='green')
    #plt.plot([4,38],[22,8],[4,31],[4,34],color='black',linewidth=4)
    # plt.plot([1,11],[2,13],color='black',linewidth=4)     #coordonnée des pistes (A,G)(J,N)
    # plt.plot([10,28],[2,8],color='black',linewidth=4)
    # plt.plot([20,18],[15,0],color='black',linewidth=4)

def AjoutChemin(chemin):
    x=[]
    y=[]
    for k in range(len(chemin)):
        x.append(xy[chemin[k]][0])
        y.append(xy[chemin[k]][1])
    plt.plot(x,y,color='red')

def DessinChemin(L,xy,chemin):
    Graphe2(xy,L)
    AjoutChemin(chemin)
    plt.show()

def DessinSuitee(L,xy):
    for k in range(len(cheminavion)):
        DessinChemin(L,xy,cheminavion[k])

## Recherche route de collision SANS adaptation

global avions
avions=[]
global cheminavion
cheminavion=[]
global distanceavion
distanceavion=[]


def AjoutDepartCible(depart,cible,numvol,tolerance):
    n=0
    CreationGraphe(L)
    depart=rd.choice(depart)
    cible=rd.choice(cible)
    nvchemin=plusCourtChemin ( depart , cible )
    listechemin=[nvchemin]
    #print('nvchemin=',nvchemin)
    for k in range(len(cheminavion)):
        for i in range(1,len(nvchemin)):
            if RechercheRouteCollision3(nvchemin[i],cheminavion[k],nvchemin,i,tolerance):
                if not RechercheRouteUnique(intersec_prec):
                    #print('route unique a l\'intersection',intersec_prec)
                    #print('nvchemin=',nvchemin)
                    graphe=ModifGraphe2()
                    #print('sortie modif graphe')
                    if plusCourtChemin(depart,cible):
                        nvchemin=plusCourtChemin(depart,cible)
                        if nvchemin:
                            listechemin.append([nvchemin])
    avions.append([numvol,depart,cible,10,-1 , nvchemin[1],-1,dt])
    cheminavion.append(nvchemin)
    temp=[]
    for k in range(0,len(nvchemin)-1):
        temp.append(CalculDistance([nvchemin[k],nvchemin[k+1]],L))
    distanceavion.append(temp)
    #print('distanceavion=', distanceavion)


def PlusieursAvions(n,dep,arr,lnumvol):
    for k in range(n):
        AjoutDepartCible(dep,arr,lnumvol,250)

def RechercheRouteCollision2(nvchemin,L,tolerance):
    #print('entrée dans la recherche coll')
    global intersec_prob
    intersec_prob=0
    global intersec_prec
    intersec_prec=0
    for k in range(len(cheminavion)):
        for i in range(1,len(nvchemin)):
            if nvchemin[i] in cheminavion[k]:
                #print('collision detectee à nvchemin=',nvchemin[i],'en i=',i,'avec l\'avion',k,'de route',cheminavion[k])
                intersec_prob=nvchemin[i]
                intersec_prec=nvchemin[i-1]
                chemin_index=cheminavion[k].index(nvchemin[i])
                collise=cheminavion[k]
                ecart=CalculDistance(collise[:chemin_index],L)-CalculDistance(nvchemin,L)
                #print('ecart=',abs(ecart))
                if abs(ecart)<tolerance:
                    #print('collision possible')
                    return True
                #else:
                    #print('enfaite c ok')
    #print('pas de collision')
    return False

def RechercheRouteCollision3(nvchemin_i,cheminavion_k,nvchemin,i,tolerance):
    global intersec_prec
    intersec_prec=0
    global intersec_prob
    intersec_prob=0
    if nvchemin_i in cheminavion_k:
        intersec_prob=nvchemin_i
        intersec_prec=nvchemin[i-1]
        chemin_index1=nvchemin.index(nvchemin_i)
        chemin_index2=cheminavion_k.index(nvchemin_i)
        ecart=CalculDistance(nvchemin[:chemin_index1],L)-CalculDistance(cheminavion_k[:chemin_index2],L)
        if abs(ecart)<tolerance:
            return True
        else:
            return False
    return False

def CalculDistanceIntersection(itnersec,chemin):
    if intersec not in chemin:
        return False
    else:
        indice = chemin.index(intersec)
        return CalculDistance(chemin[:indice+1])

def RechercheRouteUnique(intersection):
    n=0
    for k in range(len(graphe[intersection])):
        if graphe[intersection][k]!=inf and graphe[intersection][k]!=0:
            n+=1
    if n>1:
        return False
    else:
        return True

def ModifGraphe():
    # print('modification du graphe')
    # print('intersec_prec=',intersec_prec)
    # print('grapheintersecprob=',graphe[intersec_prec])
    graphe[intersec_prob][intersec_prec]=inf
    graphe[intersec_prec][intersec_prob]=inf
    #print('grapheintersecprob=',graphe[intersec_prec])

def ModifGraphe2():
    #print('graphe de l\'intersec_prec=',graphe[intersec_prob])
    graphe[intersec_prob][intersec_prec]=inf
    graphe[intersec_prec][intersec_prob]=inf
    #print('graphe de l\'intersec_prec=',graphe[intersec_prob])
    return graphe


def CalculDistance(chemin,L):
    dist=0
    for k in range(len(chemin)-1):
        if chemin[k+1] in L[chemin[k]]:
            dist+=distance[chemin[k]][L[chemin[k]].index(chemin[k+1])]
    return dist

## Evolution
last_actu = 0
dt = 0
def ActualisationPosition(tps_act):
    global last_actu
    global cheminavion , distanceavion , avions
    stop = False
    delta_t = tps_act - last_actu
    distanceavion , cheminavion , avions = RetireListesVides()
    for k in range(len(distanceavion)):
        vitesse = avions[k][3]
        delta_d = vitesse*delta_t
        if len(distanceavion[k]) != 0:
            while len(distanceavion[k]) != 0 and delta_d >= distanceavion[k][0]:
                delta_d -= distanceavion[k][0]
                distanceavion[k].pop(0)
                cheminavion[k].pop(0)
                if len(cheminavion[k]) >= 2:
                    avions[k][4] = cheminavion[k][0]
                    avions[k][5] = cheminavion[k][1]
                elif len(cheminavion[k]) == 1:
                    avions[k][4] = cheminavion[k][0]
                    avions[k][5] = -1
            if len(distanceavion[k]) != 0:
                distanceavion[k][0] -= delta_d
            if len(cheminavion[k]) >= 2:
                avions[k][4] = cheminavion[k][0]
                avions[k][5] = cheminavion[k][1]
            elif len(cheminavion[k]) == 1:
                avions[k][4] = cheminavion[k][0]
                avions[k][5] = -1
    ActualiseTempsArrivee()
    last_actu = tps_act

def Simulation(n,tolerance):
    global dt
    compt = 1
    dernier_parti = 0
    AjoutDepartCible(dep,arr,'numvol',tolerance)
    while len(distanceavion) != 0:
        if compt < n and dt != dernier_parti: # fait partir tous les avions à 1s d'intervalle
            AjoutDepartCible(dep,arr,'numvol',tolerance)
            dernier_parti = dt
            compt += 1
        ActualisationPosition(dt)
        print('distanceavion =', distanceavion)
        dt += 1

def ActualiseTempsArrivee():
    global avions
    for k in range(len(distanceavion)):
        somme = sum(distanceavion[k])
        temps = somme / avions[k][3]
        avions[k][6] = dt+temps

# avions = [0numvol , 1depart , 2cible , 3vitesse , 4last_intersec , 5next_intersec , 6temps_arrivee , 7temps_depart]
## Fonction Aide

def Reset():
    global cheminavion,distanceavion,avions
    cheminavion,distanceavion,avions = [],[],[]

def RetireListesVides():
    global date_depart
    distance_temp = []
    chemin_temp = []
    avions_temp = []
    for k in range(len(distanceavion)):
        if len(distanceavion[k]) != 0:
            distance_temp.append(distanceavion[k])
            chemin_temp.append(cheminavion[k])
            avions_temp.append(avions[k])
    # print('distance_temp =', distance_temp)
    return distance_temp,chemin_temp,avions_temp
