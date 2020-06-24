##############################################################
# Import des constantes du robot
##############################################################

import math as m
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#spec robot
Esp = 0.3           # Espacement entre les roues = 40cm
Long_robot = 0.4    # Longueur du robot
Largeur_roue = 0.02 # Largeur des roues
r = 0.04            # Rayon des roues = 4cm
Omegamax = 10.5     # En rad.s-1
Epsilonmax = 12.5   # En rad.s-2
Vmax = Omegamax * r
Amax = Epsilonmax * r

#code de la route
Limit_v = 1         # En m.s-1
Limit_a = 0.5       # En m.s-2
T0 = 10e-3          # En s






##########################################################################################################################################################################################
##########################################################################################################################################################################################

                                                        # Partie génération de trajectoire (commande avec mouvement, points et option de lissage)

##########################################################################################################################################################################################
##########################################################################################################################################################################################


##############################################################
# Commande générale du robot
##############################################################

def commande_robot():

    file = open("/Users/tomethortense/Documents/Cours/Confinement/CODEVSI/entree.txt", "r")

    line = file.readline()

    if line.find("commande")!=-1:            # Gestion des commandes avec mouvements ou avec suites de coordonnées
        index_debut = line.find("(")
        index_fin = line.find(")")
        parametre = line[index_debut+1:index_fin]

        if parametre == 'pts':
            consigne = decryptage_points(file)
        elif parametre == 'mvt':
            consigne = decryptage_mouvement(file)

    return (consigne)


##############################################################
# Commander avec des mouvements
##############################################################

def decryptage_mouvement(file):

    enter = np.zeros(((1,2)))
    m = 0
    t_init = 0
    lissage = False

    for line in file:                                       # Parcourt des lignes du fichier

        if m == 0 and line.find("lissage")!=-1:             # Gestion du lissage
            index_debut = line.find("(")
            index_fin = line.find(")")
            parametre = line[index_debut+1:index_fin]

            if parametre == 'oui':
                lissage = True
            elif parametre == 'non':
                lissage = False

        else:
            new_enter = np.zeros(((1,2)))                     # Gestion générale
            index_debut = line.find("(")
            index_fin = line.find(")")
            parametre = line[index_debut+1:index_fin]

        m = m + 1

        if (line.find("LIN")!=-1) or (line.find("lin")!=-1):    # Gestion du mvt lin
            if (isfloat(parametre)):
                new_enter = lin(float(parametre))

        if (line.find("ROT")!=-1) or (line.find("rot")!=-1):    # Gestion du mvt rot
            if (isfloat(parametre)):
                new_enter = rot(float(parametre))

        if (line.find("CIRC")!=-1) or (line.find("circ")!=-1):   # Gestion du mvt circ
            separateur = parametre.find(",")
            if (separateur != -1) :
                parametre_1 = parametre[0:separateur]
                parametre_2 = parametre[separateur+1:]
                if (isfloat(parametre_1) and isfloat(parametre_2)):
                    new_enter = circ(float(parametre_1), float(parametre_2))

        if m > 1:
            enter = np.concatenate((enter, new_enter), axis=0)        # Gestion de la multiplicité des commandes

        if m > 2 and lissage:                                     # Application du lissage
            enter, t_init = lisser_trajectoire(enter, t_init)

    retourner_fichier_txt(enter)                                  # Retours d'un fichier texte avec l'ensemble des vitesses en rad.s-1
    adaptateur_arduino(enter)                                     # Retours d'un fichier texte avec l'ensemble des vitesses adaptées à la lecture depuis Arduino
    file.close()

    return (enter)


def isfloat(str): # { similar to isdecimal() for float
	try:
		float(str)
	except ValueError:
		return False
	return True


##############################################################
# Commander avec des points
##############################################################

def decryptage_points(file):

    enter = np.zeros(((1,2)))
    mat_co = np.zeros((2,2))
    alpha = np.inf
    angle = 90
    m = 0
    t_init = 0
    lissage = False

    for line in file:                                       # Parcourt de l'ensemble des lignes du fichier

        if m == 0 and line.find("lissage")!=-1:             # Gestion du lissage
            index_debut = line.find("(")
            index_fin = line.find(")")
            parametre = line[index_debut+1:index_fin]

            if parametre == 'oui':
                lissage = True
            elif parametre == 'non':
                lissage = False

        m = m + 1

        if m > 1:
            separateur = line.find(",")
            new_enter_rot = np.zeros((1,2))
            new_enter_lin = np.zeros((1,2))

            mat_co[0][0], mat_co[0][1] = mat_co[1][0], mat_co[1][1]
            mat_co[1][0], mat_co[1][1] = float(line[:separateur]), float(line[separateur + 1:])

            phi, longueur, alpha = from_points_to_mvt(mat_co, alpha)
            new_enter_rot = rot(angle - phi*180/np.pi )
            new_enter_lin = lin(longueur)

            angle = phi*180/np.pi
            enter = np.concatenate((enter, new_enter_rot), axis=0)
            enter = np.concatenate((enter, new_enter_lin), axis=0)

        if m > 2 and lissage:
            enter, t_init = lisser_trajectoire(enter, t_init)

    retourner_fichier_txt(enter)
    adaptateur_arduino(enter)
    file.close()

    return (enter)

def from_points_to_mvt (mat_co, alpha):                                 # Permet de passer des coordonées d'un point de départ et d'un point d'arrivée, à un mouvement rot et lin

    if (mat_co[1][0] - mat_co[0][0]) == 0:                              # Gestion pour deux points avec le même x
        if mat_co[1][1] > mat_co[0][1]:
            phi = np.pi/2
        else:
            phi = - np.pi/2

    else :
        phi = np.arctan((mat_co[1][1] - mat_co[0][1]) / (mat_co[1][0] - mat_co[0][0]))

    longueur = np.sqrt((mat_co[1][1] - mat_co[0][1])**2 + (mat_co[1][0] - mat_co[0][0])**2)

    if (mat_co[1][1] > alpha * (mat_co[1][0] - mat_co[0][0]) + mat_co[0][1]) and ((mat_co[1][0] - mat_co[0][0]) != 0):   # Gestion d'un mouvement vers la partie gauche du robot car arctan a pour ensemble de définition (-pi/2, pi/2)
        phi = phi + np.pi

    if (mat_co[1][0] - mat_co[0][0]) == 0:                              # Gestion pour deux points avec le même x
        alpha = np.inf
    else:
        alpha = (mat_co[1][1] - mat_co[0][1]) / (mat_co[1][0] - mat_co[0][0])

    return (phi, longueur, alpha)


##############################################################
# The 3 mouvement generation functions
##############################################################

def lin (dist):
    if (dist>0):                        # Gestion de la marche avant/arrière
        a = 1
    else :
        a = -1

    T = abs(dist/Vmax)
    tau = Vmax/Amax

    if (T<tau) :
        tau = np.sqrt(abs(dist/Amax))
        T = tau
    nblin = int((T+tau)/T0)
    tab = np.zeros((nblin,2))

    for k in range (0,nblin):

        if (k*T0 <= tau):                       # Periode d'accélération
            w = a*(Amax*k*T0)/r
        elif (k*T0 > tau and k*T0 < T):         # Periode à vitesse constante
            w = a * (Vmax)/r
        else :                                  # Periode de décélération
            w = a * (-Amax*k*T0 + Amax*(T+tau))/r

        tab[k][0] = w
        tab[k][1] = w

    return tab


def rot (ang):

    if (ang > 0):                           # Gestion de la marche avant/arrière
        a = 1
    else :
        a = -1

    T = (abs(ang)*(np.pi/360)*Esp)/Vmax
    tau = Vmax/Amax

    if (T<tau) :
        tau = np.sqrt((abs(ang)*(np.pi/360)*Esp)/Amax)
        T = tau
    nblin = int((T+tau)/T0)
    tab = np.zeros((nblin,2))

    for k in range (0,nblin):

        if (k*T0<tau):
            w = a * (Amax*k*T0)/r
        elif (k*T0 >= tau and k*T0 <= T):
            w = a * (Vmax)/r
        else :
            w = a * (-Amax*k*T0 + Amax*(T+tau))/r

        tab[k][0] = w
        tab[k][1] = -w

    return tab


def circ (ang, rayon):

    if (ang > 0):                           # Gestion de la marche avant/arrière
        a = 1
    else :
        a = -1

    T = (abs(ang)*(np.pi/360)*(2*abs(rayon) + Esp))/Vmax
    tau = Vmax/Amax

    if (T<tau) :
        tau = np.sqrt((abs(ang)*(np.pi/360)*(2*abs(rayon) + Esp))/Amax)
        T = tau

    nblin = int((T+tau)/T0)
    tab = np.zeros((nblin,2))
    eta = (2*abs(rayon) - Esp)/(2*abs(rayon) + Esp)

    for k in range (0,nblin):

        if (k*T0 < tau):
            w1 = a * (Amax*k*T0)/r
        elif (k*T0 >= tau and k*T0 <= T):
            w1 = a * (Vmax)/r
        else :
            w1 = a * (-Amax*k*T0 + Amax*(T+tau))/r

        w2 = eta*w1

        if (rayon > 0):
            tab[k][0] = w1
            tab[k][1] = w2
        else :
            tab[k][0] = w2
            tab[k][1] = w1

    return tab


##############################################################
# Lisseur de trajectroire
##############################################################

def lisser_trajectoire(mat_vitesse, t_init):

    mat_vit_pos = abs(mat_vitesse)
    t1 = t_init

    while mat_vit_pos[t1+1][0] >= mat_vit_pos[t1][0]:  # On détecte le départ de la phase de décélaration du premier trapèze
        t1 = t1 + 1

    t2 = t1

    while mat_vit_pos[t2+1][0] <= mat_vit_pos[t2][0]:  # On détecte le départ de la phase d'accélération du deuxième trapèze
        t2 = t2 + 1

    t3 = t2 + 1

    while mat_vit_pos[t3+1][0] > mat_vit_pos[t3][0]:   # On détecte la fin de la phase d'accélération du deuxième trapèze
        t3 = t3 + 1

    delta_t1, delta_t2 = t2 - t1, t3 - t2

    if delta_t1 > delta_t2 :                            # Distinction des cas : phase de décélaration du premier trapèze plus longues que la phase d'accélération du deuxième trapèze
        new_mat_vit = np.zeros((len(mat_vitesse) - delta_t2, 2))
        new_mat_vit[:t2-delta_t2], new_mat_vit[t2:] = mat_vitesse[:t2-delta_t2], mat_vitesse[t3:]

        for i in range (delta_t2):
            new_mat_vit[t2 - delta_t2 + i] = mat_vitesse[t2 - delta_t2 + i] + mat_vitesse[t2 + i]     # On superpose les phases de décélération et d'accélération

    else :                                              # phase de décélaration du premier trapèze plus courtes que la phase d'accélération du deuxième trapèze
        new_mat_vit = np.zeros((len(mat_vitesse) - delta_t1, 2))
        new_mat_vit[:t1], new_mat_vit[t2:] = mat_vitesse[:t1], mat_vitesse[t2+delta_t1:]
    
        for i in range (delta_t1):
            new_mat_vit[t1 + i] = mat_vitesse[t1 + i] + mat_vitesse[t2 + i]                           # On superpose les phases de décélération et d'accélération

    return (new_mat_vit, t2)


##############################################################
# Fonctions permettant de retourner des fichiers txt (pour arduino (entre 0 et 255) et pour nous (vitesse réelle du moteur)
##############################################################

def adaptateur_arduino(mat_vitesse):                # On remplit le fichier texte avec des valeurs de vitesse entières comprises entre 0 et 255, avec 127 comme valeur pour 0 rad.s-1

    f = open ("/Users/tomethortense/Documents/Cours/Confinement/CODEVSI/sortie_arduino.txt", "w")

    for a in (0,1):

        for k in range (len(mat_vitesse)):

            if k == len(mat_vitesse[:,a])-1:

                if mat_vitesse[k,a] == 0:
                    texte = "%s" % (127)
                elif mat_vitesse[k,a] > 0:
                    texte = "%s" % (int(mat_vitesse[k,a]*255/21) + 128)
                else:
                    texte = "%s" % (int(mat_vitesse[k,a]*255/21) + 127)

            else:

                if mat_vitesse[k,a] == 0:
                    texte = "%s, " % (127)
                elif mat_vitesse[k,a] > 0:
                    texte = "%s, " % (int(mat_vitesse[k,a]*255/21) + 128)
                else:
                    texte = "%s, " % (int(mat_vitesse[k,a]*255/21) + 127)

            f.write(texte)

        f.write('\n')

    f.close()


def retourner_fichier_txt(mat_vitesse):             # On retourne un fichier texte avec des vitesses en rad.s-1

    f = open ("/Users/tomethortense/Documents/Cours/Confinement/CODEVSI/sortie_vitesse.txt", "w")

    for vitesse in mat_vitesse:
        texte = "%s  ;  %s\n" % (round(vitesse[0], 3), round(vitesse[1], 3))
        f.write(texte)

    f.close()







