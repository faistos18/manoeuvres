#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

void rotation(double x, double y, double xo, double yo, double theta, double &xr, double &yr)
{
    //rotate x,y around xo,yo by theta (rad)
    xr = cos(theta) * (x - xo) - sin(theta) * (y - yo) + xo;
    yr = sin(theta) * (x - xo) + cos(theta) * (y - yo) + yo;
}

enum MOUVEMENT
{
    AVANCE = 0,
    RECULE = 1,
    AVANT_DROIT = 2,
    AVANT_GAUCHE = 3,
    ARRIERE_DROIT = 4,
    ARRIERE_GAUCHE = 5,
    STARTING = 6
};

string MOUVEMENTS_TEXTE[7] = {
    "AVANCE",
    "RECULE",
    "AVANT_DROIT",
    "AVANT_GAUCHE",
    "ARRIERE_DROIT",
    "ARRIERE_GAUCHE",
    "STARTING",
};

class PointAngle
{
public:
    double x;
    double y;
    double angleRad;
    double distanceParcourue;
    double estimation;
    MOUVEMENT mouvement;
    PointAngle *pere;

    PointAngle() : x(0), y(0), angleRad(0), distanceParcourue(0), estimation(0), pere(NULL), mouvement(STARTING)
    {
    }
    PointAngle(double _x, double _y, double _angleRad) : x(_x), y(_y), angleRad(_angleRad), distanceParcourue(0), estimation(0), pere(NULL), mouvement(STARTING)
    {
    }

    PointAngle(double _x, double _y, double _angleRad,
               double _distanceParcourue, MOUVEMENT _mouvement, PointAngle *_pere)
        : x(_x), angleRad(0.f), y(_y), distanceParcourue(_distanceParcourue), estimation(0.f), mouvement(_mouvement), pere(_pere)
    {
        if (_angleRad <= -(M_PI))
        {
            angleRad = _angleRad + 2.0 * M_PI;
        }
        else if (_angleRad > M_PI)
        {
            angleRad = _angleRad - 2.0 * M_PI;
        }
        else
        {
            angleRad = _angleRad;
        }
    }
};

void transformeButVersRefBraquage(const PointAngle &pointAngle, const PointAngle &butPointAngle, double rayonBraquage, double &xOut, double &yOut)
{
    xOut = butPointAngle.x;
    yOut = butPointAngle.y;
    // translation
    xOut -= pointAngle.x;
    yOut -= pointAngle.y;
    // scale
    xOut /= rayonBraquage;
    yOut /= rayonBraquage;
    // rotation
    // butPointAngleAngle = butPointAngleAngle % (math.pi /2) # faux je pense. Les symétries en angle, pas important pour l'instant
    // double butPointAngleAngle = butPointAngle.angleRad - pointAngle.angleRad;

    rotation(xOut, yOut, 0, 0, -pointAngle.angleRad, xOut, yOut);
}

// LA FONCTION POUR XAVIER

void chercherMeilleureManoeuvreSansAngle(PointAngle &pointAngle, const PointAngle &butPointAngle, bool prefererArriere, double rayonBraquage, MOUVEMENT *mouvement1, double *outDist1, MOUVEMENT *mouvement2, double *outDist2, double *outAngleArrivee)
{
    double butX, butY;

    transformeButVersRefBraquage(pointAngle, butPointAngle, rayonBraquage, butX, butY);
    // symétrie gauche / droite avant / arriere
    bool inaccessible = (butX * butX + (abs(butY) - 1.0) * (abs(butY) - 1.0)) < 1.0; //dans le cercle rayon de braquage
    bool symetrieX = butX < 0, symetrieY = butY < 0;
    if (inaccessible)
    {
        if (symetrieY)
        {
            if (symetrieX && !prefererArriere)
            {
                *mouvement1 = AVANT_GAUCHE;
                *mouvement2 = ARRIERE_DROIT;
            }
            else
            {
                *mouvement1 = ARRIERE_GAUCHE;
                *mouvement2 = AVANT_DROIT;
            }
        }
        else
        {
            if (symetrieX && !prefererArriere)
            {
                *mouvement1 = AVANT_DROIT;
                *mouvement2 = ARRIERE_GAUCHE;
            }
            else
            {
                *mouvement1 = ARRIERE_DROIT;
                *mouvement2 = AVANT_GAUCHE;
            }
        }
    }
    else
    {
        if (symetrieX)
        {
            if (symetrieY)
                *mouvement1 = ARRIERE_DROIT;
            else
                *mouvement1 = ARRIERE_GAUCHE;
            *mouvement2 = RECULE;
        }
        else
        {
            if (symetrieY)
                *mouvement1 = AVANT_DROIT;
            else
                *mouvement1 = AVANT_GAUCHE;
            *mouvement2 = AVANCE;
        }
    }
    butY = abs(butY);
    if (!inaccessible || !prefererArriere)
    {
        butX = abs(butX);
    }

    if (inaccessible)
    {
        //symétrie gauche / droite
        butY = abs(butY);

        // dans le cercle, on peut calculer les manoeuvres.
        // addition d'une manoeuvre où on recule, une où on avance. dist1, dist2
        // on utilise forcément le côté "vert", donc une rotation
        // C => centre du cercle du bas, (0, -1)
        // l = vecteur C => But
        double lY = butY + 1.0;
        double l = sqrt(butX * butX + lY * lY);

        double angleL = atan2(lY, butX) - M_PI / 2.0;
        // because reference is y axis, not x axis
        // al kashi : c² = b² + a² - 2ab*gamma (gamma1 = angle F-C-But)
        // => gamma1 = acos( (a²+b²-c²)/2ab)
        // or b = 2, a = 1, c = l
        // on obtient gamma1 = acos( (l²+3)/4l)
        double gamma1 = acos((l * l + 3.0) / (4.0 * l));
        //  on réapplique Al kashi avec l'autre angle
        //  a=1, b = 2, c=l, on obtient
        //  acos( (5-l²) / 4)
        double gamma2 = acos((5.0 - l * l) / 4.0);
        *outDist1 = gamma1 + angleL;
        *outDist2 = gamma2;
        *outAngleArrivee = gamma1 + gamma2 + angleL;
    }
    else // accessible
    {
        double longueurL = sqrt(butX * butX + (butY - 1.0) * (butY - 1.0));
        double longueurSegment = sqrt(longueurL * longueurL - 1.0);

        double angleL = atan2(butY - 1.0, butX);
        double angleTriangle = acos(1.0 / longueurL);
        double angle = angleL - angleTriangle + (M_PI / 2.0);

        *outDist1 = angle;
        *outDist2 = longueurSegment;
        *outAngleArrivee = angle;
    }
    // correction scaling
    *outDist1 *= rayonBraquage;
    *outDist2 *= rayonBraquage;
    //symetries
    if (symetrieY)
        *outAngleArrivee = -(*outAngleArrivee);

    if (symetrieX && !(inaccessible && prefererArriere))
        *outAngleArrivee = -(*outAngleArrivee);
}

struct MouvementPendant
{
    MOUVEMENT mouvement;
    double distance;
};

MouvementPendant complementaire(const MouvementPendant &mp)
{
    if (mp.mouvement == AVANCE || mp.mouvement == RECULE)
    {
        throw "pas de complementaire pour avance et recule";
    }
    else
    {
        MouvementPendant res;
        if (mp.mouvement == AVANT_DROIT)
            res.mouvement = ARRIERE_DROIT;
        else if (res.mouvement == AVANT_GAUCHE)
            res.mouvement = ARRIERE_GAUCHE;
        else if (res.mouvement == ARRIERE_DROIT)
            res.mouvement = AVANT_DROIT;
        else //ARRIERE_GAUCHE
            res.mouvement = AVANT_GAUCHE;
        // A tester + blinder ! surement faux !
        res.distance = 2.0 * M_PI - mp.distance;
        return res;
    }
}
typedef vector<MouvementPendant> Manoeuvre;

/*
Depart : D. Arrivée : A.
Il y a deux points angles, donc 4 CIR. D1, D2, A1, A2.
Donc 4 couples de cercles (D1A1, D1A2, D2A1, D2A2)
Chaque couple a deux tangentes aux deux cercles.
Si les deux cercles n'ont aucune surface commune ( distance entre les centres > 2 rayons), alors on a aussi 2 tangentes croisées.
Ca nous fait donc 4*4 = 16 manoeuvres de type tourner => avancer(ou reculer) => tourner.
Sauf que, pour atteindre un point sur un CIR, on peut y aller en avant droit, ou en arrière droit (on passe par l'autre côté), donc.
Donc on a encore un facteur 4, donc 64 manoeuvres. MAIS ces manoeuvres sont forcément plus longues que si on passe à chaque fois par l'autre côté.
MAIS si on veut absolument commencer par reculer/avancer ou éviter de passer sur une zone, elles peuvent être intéressantes, donc on peut les inclure, ou pas.


Il y a 8 tangentes 

d'abord en python : je vais décomposer une manoeuvre en plein de petites et les afficher toutes. Afficher aussi les cercles ?
OUI, parce que c'est pas encore totalement clair ces histoires de sens.
J'ai l'impression que les croisés sont toujours plus longues ET inversent le sens.

*/
vector<Manoeuvre> manoeuvresSansAngles(
    const PointAngle &depart, const PointAngle &arrivee,
    double rayonBraquage = 1.90,
    bool departQueEnArriere = false,
    bool departQueEnAvant = false,
    bool passerParAutreCoteDepart = false,
    bool passerParAutreCotéArrivee = false,
    bool seulementLaPlusCourte= false)
{
    //Deja, on se met dans le referentiel ou pa1 = (0,0,0) ?
    //Et rayonDeBraquage = 1.0
    //Comme ça, on a déjà ses cercles.

    if (passerParAutreCoteDepart)
    {
        //le complementaire
        //pour chaque manoeuvre, on prends le mouvement le MouvementPendant du depart, et on le fait dans le sens inverse.
    }
    if (passerParAutreCotéArrivee)
    {
        //Pareil à l'arrivee
    }
}

vector<Manoeuvre> tangentesDeuxCIR(bool pa1CirDroite, PointAngle &pa2, bool pa2cirDroite, bool inclureTangentesCroisees = true)
{
    //déjà : coordonnées des centres
    // puis
    if (inclureTangentesCroisees)
    {
        //trouver centre des deux cercles.
        // trouver angle alpha acos (2/L) ou L = distance deux centres
    }
}

int main()
{
    cout << " Calculer manoeuvre sans angle" << endl;
    double x, y;
    cout << " x ? " << endl;
    cin >> x;
    cout << " y ? " << endl;
    cin >> y;

    bool prefererArriere = true;
    cout << " preferer en arriere ? 0/1" << endl;
    cin >> prefererArriere;
    double distance1, distance2, angleArrivee;
    MOUVEMENT mouvement1, mouvement2;
    PointAngle depart(0, 0, 0);
    chercherMeilleureManoeuvreSansAngle(depart, PointAngle(x, y, 0), prefererArriere, 1.90, &mouvement1, &distance1, &mouvement2, &distance2, &angleArrivee);
    cout << "mouvement 1 : " << MOUVEMENTS_TEXTE[mouvement1] << " avec distance " << distance1 << endl;
    cout << "mouvement 2 : " << MOUVEMENTS_TEXTE[mouvement2] << " avec distance " << distance2 << endl;
    cout << "arrivee avec un angle de " << (angleArrivee * 180.0 / M_PI) << " deg" << endl;

    return 0;
}
