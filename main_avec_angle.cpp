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


// Pour xavier : 
// j'ai gardé cette classe avec des utilitaires dedans pour pouvoir effectuer vraiment les manoeuvres
// et donc vérifier que j'arrive bien au point d'arrivée.
// mais techniquement, la fonction getManoeuvres() n'utilise que x,y, angleRad
// donc c'est facile de remplacer ça par n'importe quelle struct avec ces trois paramètres,
// voire d'en faire une fonction qui prends tout ça en paramètre directement...

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

    void vecteurAngle(double &xr, double &yr) const
    {
        xr = cos(angleRad);
        yr = sin(angleRad);
    }
    void vecteurAngleDroite(double &xr, double &yr) const
    {
        double xr1, yr1, tmp;
        vecteurAngle(xr1, yr1);
        xr = yr1;
        yr = -xr1;
    }
    void vecteurAngleGauche(double &xr, double &yr) const
    {
        double xr1, yr1;
        vecteurAngle(xr1, yr1);
        xr = -yr1;
        yr = xr1;
    }
    PointAngle *avance(double pas)
    {
        double xr, yr;
        vecteurAngle(xr, yr);
        double xFils = x + xr * pas;
        double yFils = y + yr * pas;
        return new PointAngle(xFils, yFils, angleRad, distanceParcourue + pas, AVANCE, this);
    }
    PointAngle *recule(double pas)
    {
        double xr, yr;
        vecteurAngle(xr, yr);
        double xFils = x - xr * pas;
        double yFils = y - yr * pas;
        return new PointAngle(xFils, yFils, angleRad, distanceParcourue + pas, RECULE, this);
    }
    PointAngle *avantDroit(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = -pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleDroite(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue + pas, AVANT_DROIT, this);
    }
    PointAngle *avantGauche(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleGauche(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue + pas, AVANT_GAUCHE, this);
    }
    PointAngle *arriereDroit(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleDroite(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue + pas, ARRIERE_DROIT, this);
    }
    PointAngle *arriereGauche(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = -pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleGauche(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue + pas, ARRIERE_GAUCHE, this);
    }

    PointAngle *bouge(MOUVEMENT mouvement, double pas, double rayonBraquage, unsigned int nombreDeFois = 1)
    {
        PointAngle *resultat;
        if (mouvement == AVANCE)
            resultat = avance(pas);
        else if (mouvement == RECULE)
            resultat = recule(pas);
        else if (mouvement == AVANT_DROIT)
            resultat = avantDroit(pas, rayonBraquage);
        else if (mouvement == AVANT_GAUCHE)
            resultat = avantGauche(pas, rayonBraquage);
        else if (mouvement == ARRIERE_DROIT)
            resultat = arriereDroit(pas, rayonBraquage);
        else if (mouvement == ARRIERE_GAUCHE)
            resultat = arriereGauche(pas, rayonBraquage);
        else
            throw "mouvement inconnu";

        if (nombreDeFois == 1)
            return resultat;
        else
        {
            while (nombreDeFois != 1)
            {
                nombreDeFois--;
                PointAngle *oldResultat = resultat;
                resultat = resultat->bouge(mouvement, pas, rayonBraquage);
                delete oldResultat;
            }
            return resultat;
        }
    }
};

struct MouvementPendant
{
    MOUVEMENT mouvement;
    double distance;
};

//plusieurs mouvement = une manoeuvre
typedef vector<MouvementPendant> Manoeuvre;

MOUVEMENT inverse(const MOUVEMENT m)
{
    if (m == AVANT_DROIT)
        return ARRIERE_DROIT;
    else if (m == AVANT_GAUCHE)
        return ARRIERE_GAUCHE;
    else if (m == ARRIERE_DROIT)
        return AVANT_DROIT;
    else if (m == ARRIERE_GAUCHE)
        return AVANT_GAUCHE;
    else if (m == AVANCE)
        return RECULE;
    else
        return AVANCE;
}

// detour = au lieu d'avancer-droite d'un quart de tour, je recule-droite de trois quart de tour
Manoeuvre detour(const Manoeuvre &m, bool auDepart, bool aLArrivee)
{
    MouvementPendant mp1 = m[0];
    MouvementPendant mp3 = m[2];
    if (auDepart)
        mp1 = MouvementPendant{inverse(mp1.mouvement), 2.0 * M_PI - mp1.distance};
    if (aLArrivee)
        mp3 = MouvementPendant{inverse(mp3.mouvement), 2.0 * M_PI - mp3.distance};
    return Manoeuvre{mp1, m[1], mp3};
}

MOUVEMENT virage(bool coteGauche, bool enAvant)
{
    if (coteGauche)
    {
        if (enAvant)
            return AVANT_GAUCHE;
        else
            return ARRIERE_GAUCHE;
    }
    else
    {
        if (enAvant)
            return AVANT_DROIT;
        else
            return ARRIERE_DROIT;
    }
}

double regulAngle(double angle)
{
    if (angle < -M_PI)
        return angle + 2.0 * M_PI;
    else if (angle > M_PI)
        return angle - 2.0 * M_PI;
    else
        return angle;
}

Manoeuvre tangenteDroite(const PointAngle &pointAngleArrivee, bool departCoteGauche, bool avancerAuMilieu)
{
    double yCentreCercleDepart, xCentreCercleArrivee, yCentreCercleArrivee, xVect, yVect;
    if (departCoteGauche)
    {
        yCentreCercleDepart = 1.0;
        xVect = -sin(pointAngleArrivee.angleRad);
        yVect = cos(pointAngleArrivee.angleRad);
    }
    else
    {
        yCentreCercleDepart = -1.0;
        xVect = sin(pointAngleArrivee.angleRad);
        yVect = -cos(pointAngleArrivee.angleRad);
    }
    xCentreCercleArrivee = pointAngleArrivee.x + xVect;
    yCentreCercleArrivee = pointAngleArrivee.y + yVect;

    // segment = segment tangent d'un cercle à l'autre. Il fait la même longueur que celui qui relie les centre des deux cercles, et est parallèle, donc même angle
    double dySegment = yCentreCercleArrivee - yCentreCercleDepart;
    double longueurSegment = sqrt(xCentreCercleArrivee * xCentreCercleArrivee + dySegment * dySegment);

    double angle = atan2(dySegment, xCentreCercleArrivee); // car xSegment = 0, donc dxSegment = xCetnreCercleArrivee - 0
    MOUVEMENT mouvementMilieu = AVANCE;
    if (!avancerAuMilieu)
    {
        mouvementMilieu = RECULE;
        angle = regulAngle(angle + M_PI); // un demi tour en plus, on est obligés de repartir dans l'autre sens !
    }
    // yCentreCercleDepart est juste utilisé car 1 si cote gauche, -1 si cote droit
    MOUVEMENT mouvement1 = virage(departCoteGauche, (yCentreCercleDepart * angle) > 0.0);
    double angleArrivee = regulAngle(pointAngleArrivee.angleRad - angle);
    MOUVEMENT mouvement3 = virage(departCoteGauche, (yCentreCercleDepart * angleArrivee) > 0.0);
    return Manoeuvre{
        MouvementPendant{mouvement1, abs(angle)},
        MouvementPendant{mouvementMilieu, longueurSegment},
        MouvementPendant{mouvement3, abs(angleArrivee)}};
}

// Si depart côté gauche, arrivée côté droit, car on fait un demi-tour
// renvoie manoeuvre vide si pas de tangente !

Manoeuvre tangenteCroisee(const PointAngle &pointAngleArrivee, bool departCoteGauche, bool avancerAuMilieu)
{
    double yCentreCercleDepart, xCentreCercleArrivee, yCentreCercleArrivee, xVect, yVect;
    if (departCoteGauche)
    {
        yCentreCercleDepart = 1.0;
        xVect = sin(pointAngleArrivee.angleRad);
        yVect = -cos(pointAngleArrivee.angleRad);
    }
    else
    {
        yCentreCercleDepart = -1.0;
        xVect = -sin(pointAngleArrivee.angleRad);
        yVect = cos(pointAngleArrivee.angleRad);
    }
    xCentreCercleArrivee = pointAngleArrivee.x + xVect;
    yCentreCercleArrivee = pointAngleArrivee.y + yVect;

    double dySegment = yCentreCercleArrivee - yCentreCercleDepart;
    double longueurSegment = sqrt(xCentreCercleArrivee * xCentreCercleArrivee + dySegment * dySegment);
    double angleSegment = atan2(dySegment, xCentreCercleArrivee); // car xSegment = 0, donc dxSegment = xCetnreCercleArrivee - 0

    if (longueurSegment <= 2.0)
    {
        return Manoeuvre();
    }
    else
    {
        double angleTangente = asin(2.0 / longueurSegment); // asin d'un truc positif : entre 0 et pi / 2
        // pythagore : demiTangente**2 + 1 = (longueurSegment/2)**2
        //             demiTangente = sqrt( longueurSegment/2)**2 -1)
        double longueurTangente = 2.0 * sqrt((longueurSegment * longueurSegment / 4.0) - 1.0);

        MOUVEMENT mouvementMilieu;
        double angleDepart;
        if (avancerAuMilieu)
        {
            mouvementMilieu = AVANCE;
            angleDepart = regulAngle(angleSegment + angleTangente);
        }
        else
        {
            mouvementMilieu = RECULE;
            angleDepart = regulAngle(M_PI + angleSegment - angleTangente);
        }
        // yCentreCercleDepart est juste utilisé car 1 si depart cote gauche, -1 si depart cote droit
        MOUVEMENT mouvement1 = virage(departCoteGauche, (yCentreCercleDepart * angleDepart) > 0);

        double angleArrivee = regulAngle(pointAngleArrivee.angleRad - angleDepart);

        MOUVEMENT mouvement3 = virage(!departCoteGauche, !(yCentreCercleDepart * angleArrivee > 0));
        return Manoeuvre{
            MouvementPendant{mouvement1, abs(angleDepart)},
            MouvementPendant{mouvementMilieu, longueurTangente},
            MouvementPendant{mouvement3, abs(angleArrivee)}};
    }
}

void ajouteTangenteCroisee(const PointAngle &pointAngleArrivee, bool departCoteGauche, bool avancerAuMilieu, vector<Manoeuvre> &manoeuvres)
{
    Manoeuvre manoeuvre = tangenteCroisee(pointAngleArrivee, departCoteGauche, avancerAuMilieu);
    if (manoeuvre.size() != 0)
        manoeuvres.push_back(manoeuvre);
}

vector<Manoeuvre> getManoeuvres(
    const PointAngle &depart,
    const PointAngle &arrivee,
    double rayonBraquage = 1.90,
    bool detourDepartPossible = false,
    bool detourArriveePossible = false,
    bool departQueEnAvant = false,
    bool departQueEnArriere = false,
    bool queLaPlusCourte = false)
{
    if (departQueEnArriere && departQueEnAvant)
    {
        // throw ?
        return vector<Manoeuvre>();
    }
    double xRefBraquage = arrivee.x;
    double yRefBraquage = arrivee.y;
    //translation
    xRefBraquage -= depart.x;
    yRefBraquage -= depart.y;
    // scale
    xRefBraquage /= rayonBraquage;
    yRefBraquage /= rayonBraquage;
    //rotation
    double angleRefBraquage = arrivee.angleRad - depart.angleRad;
    rotation(xRefBraquage, yRefBraquage, 0, 0, -depart.angleRad, xRefBraquage, yRefBraquage);
    PointAngle pointAngleArriveeRefBraquage = PointAngle(xRefBraquage, yRefBraquage, angleRefBraquage);

    vector<Manoeuvre> manoeuvres;

    manoeuvres.push_back(tangenteDroite(pointAngleArriveeRefBraquage, true, true));
    manoeuvres.push_back(tangenteDroite(pointAngleArriveeRefBraquage, true, false));
    manoeuvres.push_back(tangenteDroite(pointAngleArriveeRefBraquage, false, true));
    manoeuvres.push_back(tangenteDroite(pointAngleArriveeRefBraquage, false, false));

    ajouteTangenteCroisee(pointAngleArriveeRefBraquage, true, true, manoeuvres);
    ajouteTangenteCroisee(pointAngleArriveeRefBraquage, true, false, manoeuvres);
    ajouteTangenteCroisee(pointAngleArriveeRefBraquage, true, true, manoeuvres);
    ajouteTangenteCroisee(pointAngleArriveeRefBraquage, true, true, manoeuvres);

    // ajouter les variations avec detour avant / arriere
    unsigned int nbrManoeuvresDeBase = manoeuvres.size();
    for (unsigned int i = 0; i < nbrManoeuvresDeBase; i++)
    {
        if (detourDepartPossible)
            manoeuvres.push_back(detour(manoeuvres[i], true, false));
        if (detourArriveePossible)
            manoeuvres.push_back(detour(manoeuvres[i], false, true));
        if (detourDepartPossible && detourArriveePossible)
            manoeuvres.push_back(detour(manoeuvres[i], true, true));
    }

    // rescale tout par rayonBraquage, et filtrage
    vector<Manoeuvre>::iterator m = manoeuvres.begin();
    while (m != manoeuvres.end())
    {
        // attention, si le mouvement est (AVANT_DROIT,0), (RECULE, 3), (...), alors techniquement on a un départ en avant...
        // donc même si on veut que des départs en avant, il apparait toujours...
        // donc faut-il filtrer si la première distance est 0 ?
        // mais remarque, on peut aussi avoir un reculage de 1 centimètre avant d'avancer ? est-ce que c'est toujoursun départ en arrière ?
        // faut-il rajouter un seuil ? Si oui combien :-/
        // ça dépendra de l'usage de Xavier
        if ((!departQueEnArriere && !departQueEnAvant) || (departQueEnArriere && ((*m)[0].mouvement == ARRIERE_DROIT || (*m)[0].mouvement == ARRIERE_GAUCHE)) || (departQueEnAvant && ((*m)[0].mouvement == AVANT_DROIT || (*m)[0].mouvement == AVANT_GAUCHE)))
        {
            for (unsigned i = 0; i < m->size(); i++)
            {
                (*m)[i].distance = (*m)[i].distance * rayonBraquage;
            }
            ++m;
        }
        else
        {
            // erase() invalidates the iterator, use returned iterator
            m = manoeuvres.erase(m);
        }
    }

    if (queLaPlusCourte)
    {
        Manoeuvre meilleureManoeuvre;
        double meilleureDistance = INFINITY;
        double distance;
        for (m = manoeuvres.begin(); m != manoeuvres.end(); m++)
        {
            distance = (*m)[0].distance + (*m)[1].distance + (*m)[2].distance;
            if (distance < meilleureDistance)
            {
                meilleureDistance = distance;
                meilleureManoeuvre = *m;
            }
        }
        return vector<Manoeuvre>{meilleureManoeuvre};
    }
    else
    {
        return manoeuvres;
    }
}

int main()
{
    const double rayonBraquage = 1.90;
    PointAngle depart(0, 0, 0); //Devrait etre const, mais osef, c'est pour tester ici
    const double epsilon = 0.0001;

    cout << " Calculer manoeuvre avec angle" << endl;
    double x, y, angle;
    cout << " x ? " << endl;
    cin >> x;
    cout << " y ? " << endl;
    cin >> y;
    cout << " angle (deg) ? " << endl;
    cin >> angle;
    angle *= M_PI / 180.0;

    bool detourDepartPossible = true,
         detourArriveePossible = true,
         departQueEnAvant = false,
         departQueEnArriere = true,
         queLaPlusCourte = true;

    cout << "detourDepartPossible ? 0/1" << endl;
    cin >> detourDepartPossible;
    cout << "detourArriveePossible ? 0/1" << endl;
    cin >> detourArriveePossible;
    cout << "departQueEnAvant ? 0/1" << endl;
    cin >> departQueEnAvant;
    cout << "departQueEnArriere ? 0/1" << endl;
    cin >> departQueEnArriere;
    cout << "queLaPlusCourte ? 0/1" << endl;
    cin >> queLaPlusCourte;
    PointAngle arrivee(x, y, angle);

    vector<Manoeuvre> manoeuvres = getManoeuvres(depart, arrivee, rayonBraquage,
                                                 detourDepartPossible, detourArriveePossible, departQueEnAvant, departQueEnArriere, queLaPlusCourte);

    for (unsigned int i = 0; i < manoeuvres.size(); i++)
    {
        Manoeuvre &manoeuvre = manoeuvres[i];
        double distance = manoeuvre[0].distance + manoeuvre[1].distance + manoeuvre[2].distance;
        cout << "manoeuvre avec distance totale de " << distance << endl;
        cout << "   mouvement 1 : " << MOUVEMENTS_TEXTE[manoeuvre[0].mouvement] << " avec distance " << manoeuvre[0].distance << endl;
        cout << "   mouvement 2 : " << MOUVEMENTS_TEXTE[manoeuvre[1].mouvement] << " avec distance " << manoeuvre[1].distance << endl;
        cout << "   mouvement 3 : " << MOUVEMENTS_TEXTE[manoeuvre[2].mouvement] << " avec distance " << manoeuvre[2].distance << endl;
        //vérification sibien arrivé ou non.
        PointAngle *etape1 = depart.bouge(manoeuvre[0].mouvement, manoeuvre[0].distance, rayonBraquage);
        PointAngle *etape2 = etape1->bouge(manoeuvre[1].mouvement, manoeuvre[1].distance, rayonBraquage);
        PointAngle *etape3 = etape2->bouge(manoeuvre[2].mouvement, manoeuvre[2].distance, rayonBraquage);
        if (abs(etape3->x - arrivee.x) < epsilon &&
            abs(etape3->y - arrivee.y) < epsilon &&
            abs(etape3->angleRad - arrivee.angleRad) < epsilon)
        {
            //cout << "bien arrive !" << endl;
        }
        else
        {
            cout << "FAIL  x  : " << etape3->x << ", y : " << etape3->y << ", angle : " << etape3->angleRad * 180.0 / M_PI << endl;
        }
        delete etape1;
        delete etape2;
        delete etape3;
    }

    double wait;
    cin >> wait;
    return 0;
}
