#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <list>
#include <algorithm>

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

const double epsilonMetres = 0.05;
const double epsilonRad = 0.08726646259971647;

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

    void vecteurAngle(double &xr, double &yr)
    {
        xr = cos(angleRad);
        yr = sin(angleRad);
    }
    void vecteurAngleDroite(double &xr, double &yr)
    {
        double xr1, yr1, tmp;
        vecteurAngle(xr1, yr1);
        xr = yr1;
        yr = -xr1;
    }
    void vecteurAngleGauche(double &xr, double &yr)
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

    bool tresProche(PointAngle p2)
    {
        return abs(x - p2.x) < epsilonMetres && abs(y - p2.y) < epsilonMetres && abs(angleRad - p2.angleRad) < epsilonRad;
    }
};

ostream &operator<<(ostream &strm, const PointAngle &pa)
{
    return strm << "PointAngle(x = " << pa.x << ", y = " << pa.y << ", angle = " << (pa.angleRad * 180.0 / M_PI)
                << ", dist = " << pa.distanceParcourue << " )";
}

string resumerManoeuvres(PointAngle *pa)
{
    vector<PointAngle *> chaineDesPeres;

    PointAngle *ancetre = pa;

    while (ancetre->pere != NULL)
    {
        chaineDesPeres.push_back(ancetre);
        ancetre = ancetre->pere;
    }
    // les vieux d'abord !
    reverse(chaineDesPeres.begin(), chaineDesPeres.end());
    // list des manoeuvres avec (manoeuvre, distanceParcourue)
    vector< pair<MOUVEMENT, double> > chaineMouvements { pair<MOUVEMENT, double>(chaineDesPeres[0]->mouvement, chaineDesPeres[0]->distanceParcourue)};
    double ancienneDistanceParcourue = chaineDesPeres[0]->distanceParcourue;

    for (unsigned int i = 1; i < chaineDesPeres.size(); i++)
    {
        double pas = chaineDesPeres[i]->distanceParcourue - ancienneDistanceParcourue;
        ancienneDistanceParcourue = chaineDesPeres[i]->distanceParcourue;
        if (chaineMouvements.back().first == chaineDesPeres[i]->mouvement)
        {
            chaineMouvements.back().second = round((chaineMouvements.back().second + pas) * 1000.0) / 1000.0;
        }
        else
        {
            chaineMouvements.push_back(pair<MOUVEMENT, double>(chaineDesPeres[i]->mouvement, round(pas * 1000.0) / 1000.0));
        }
    }

    string res = "";
    for (unsigned int i = 0; i < chaineMouvements.size(); i++)
    {
        std::ostringstream strs;
        strs << "(" << MOUVEMENTS_TEXTE[chaineMouvements[i].first] << ", " << chaineMouvements[i].second << ")";
        if (i != chaineMouvements.size() - 1)
        {
            strs << ",";
        }
        res += strs.str();
    }
    return res;
}

void testMouvements()
{
    cout << "TEST MOUVEMENTS" << endl;
    MOUVEMENT mouvements[6] = {
        AVANCE,
        RECULE,
        AVANT_DROIT,
        AVANT_GAUCHE,
        ARRIERE_DROIT,
        ARRIERE_GAUCHE,
    };
    const double pas = 0.3;
    const double rayonBraquage = 1.90;

    const unsigned int nbrMouvements = 25;
    PointAngle depart;
    for (unsigned int i = 0; i < 6; i++)
    {
        PointAngle *resultat = depart.bouge(mouvements[i], pas, rayonBraquage, nbrMouvements);
        cout << *resultat << " manoeuvres : " << resumerManoeuvres(resultat) << endl;
        delete resultat;
    }
}

double diffAngle(double a1, double a2)
{
    double a = a2 - a1;
    if (a > M_PI)
        a -= 2.f * M_PI;
    else if (a <= -M_PI)
        a += 2.f * M_PI;
    return abs(a);
}

bool butAtteint(PointAngle pointAngle, PointAngle butPointAngle, const double toleranceXY, const double toleranceAngle)
{
    return abs(pointAngle.x - butPointAngle.x) < toleranceXY &&
           abs(pointAngle.y - butPointAngle.y) < toleranceXY &&
           diffAngle(pointAngle.angleRad, butPointAngle.angleRad) < toleranceAngle;
}
double distanceCarre(const PointAngle &pa1, const PointAngle &pa2)
{
    return pow(pa2.x - pa1.x, 2.0) + pow(pa2.y - pa1.y, 2.0);
}
double distance(const PointAngle &pa1, const PointAngle &pa2)
{
    return sqrt(distanceCarre(pa1, pa2));
}

double heuristiqueAngle(const PointAngle &pointAngle, const PointAngle &butPointAngle, const double rayonBraquage)
{
    return rayonBraquage * diffAngle(pointAngle.angleRad, butPointAngle.angleRad);
}

double heuristique(const PointAngle &pointAngle, const PointAngle &butPointAngle, const double rayonBraquage, const double toleranceAngle = 0.0)
{
    return max(distance(pointAngle, butPointAngle), heuristiqueAngle(pointAngle, butPointAngle, rayonBraquage) - toleranceAngle);
}

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

void calculMouvementsInaccessible()
{
}

double heuristiqueMeilleureDistanceInaccessible(const PointAngle &pointAngle, const PointAngle &butPointAngle, double rayonBraquage, double toleranceAngle = 0)
{
    double butX, butY, distanceTotale;

    transformeButVersRefBraquage(pointAngle, butPointAngle, rayonBraquage, butX, butY);
    // symétrie avant / arriere
    butX = abs(butX);
    //  gauche / droite
    butY = abs(butY);

    bool inaccessible = (butX * butX + (butY - 1.0) * (butY - 1.0)) < 1.0; //dans le cercle rayon de braquage
    if (inaccessible)
    {
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
        double dist1 = gamma1 + angleL; // normalement : - angleL. test avec +.
        //  on réapplique Al kashi avec l'autre angle
        //  a=1, b = 2, c=l, on obtient
        //  acos( (5-l²) / 4)
        double gamma2 = acos((5.0 - l * l) / 4.0);
        distanceTotale = dist1 + gamma2;
    }
    else
    {
        double longueurL = sqrt(butX * butX + (butY - 1.0) * (butY - 1.0));
        double longueurSegment = sqrt(longueurL * longueurL - 1.0);

        double angleL = atan2(butY - 1.0, butX);
        double angleTriangle = acos(1.0 / longueurL);
        double angle = angleL - angleTriangle + (M_PI / 2.0);
        distanceTotale = angle + longueurSegment;
    }
    // correction scaling
    distanceTotale *= rayonBraquage;
    return max(heuristiqueAngle(pointAngle, butPointAngle, rayonBraquage) - toleranceAngle, distanceTotale);
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

PointAngle *prendreMeilleurCandidat(list<PointAngle *> &pointsAChercher, const PointAngle &butPointAngle, double rayonBraquage, double toleranceAngle)
{
    double meilleureEstimation = 100000000, estimation = 0;
    list<PointAngle *>::iterator meilleurCandidatIterator = pointsAChercher.begin();
    int i = 0;
    for (auto pointIterator = pointsAChercher.begin(); pointIterator != pointsAChercher.end(); pointIterator++)
    {
        estimation = 0;
        if ((*pointIterator)->estimation != 0)
        {
            estimation = (*pointIterator)->estimation;
        }
        else
        {
            estimation = (*pointIterator)->distanceParcourue + heuristiqueMeilleureDistanceInaccessible(**pointIterator, butPointAngle, rayonBraquage, toleranceAngle);
            if ((*pointIterator)->pere != NULL)
            {
                estimation = max(estimation, (*pointIterator)->pere->estimation);
            }
            (*pointIterator)->estimation = estimation;
        }

        //cout << **pointIterator << " with estimation " << estimation << endl;
        if (estimation < meilleureEstimation)
        {
            //meilleurCandidatIterator = list<PointAngle*>::iterator(pointIterator);
            meilleurCandidatIterator = pointIterator;
            meilleureEstimation = estimation;
            //cout << estimation <<endl;
            i++;
        }
    }
    //cout << "i : " << i <<endl;
    //cout << "size  : " << pointsAChercher.size() << endl;
    PointAngle *resultat = *meilleurCandidatIterator;
    pointsAChercher.erase(meilleurCandidatIterator);

    //cout << " resultat : "  << *resultat << " with estimation " << estimation << endl;
    return resultat;
}

void nettoyerListes(list<PointAngle *> l1, list<PointAngle *> l2)
{
    for (auto i = l1.begin(); i != l1.end(); i++)
    {
        delete *i;
    }
    for (auto j = l2.begin(); j != l2.end(); j++)
    {
        delete *j;
    }
}

bool chercherMeilleureManoeuvre(
    const PointAngle &butPointAngle,
    const double pas = 0.3,
    const double rayonBraquage = 1.90,
    const double toleranceXY = 0.36,
    const double toleranceAngle = (20.0 / 180.0 * M_PI),
    const int limite = -1)
{
    unsigned int iterations = 0;

    list<PointAngle *> pointsAChercher;
    pointsAChercher.push_back(new PointAngle());

    list<PointAngle *> pointsDejaCherches;

    bool abandon = false;

    while (limite == -1 || iterations < limite)
    {
        PointAngle *candidat = prendreMeilleurCandidat(pointsAChercher, butPointAngle, rayonBraquage, toleranceAngle);
        //cout << *candidat << endl;

        if (iterations % 100 == 0)
        {
            cout << iterations << " ";
        }
        for (auto mouvement = 0; mouvement < 6; mouvement++)
        {
            abandon = false;
            PointAngle *successeur = candidat->bouge(MOUVEMENT(mouvement), pas, rayonBraquage);

            if (butAtteint(*successeur, butPointAngle, toleranceXY, toleranceAngle))
            {
                cout << "but atteint en " << iterations << " iterations" << endl;
                cout << *successeur << endl;
                cout << resumerManoeuvres(successeur) << endl;
                nettoyerListes(pointsAChercher, pointsDejaCherches);
                return true;
            }
            else
            {
                for (auto it = pointsDejaCherches.begin(); it != pointsDejaCherches.end(); ++it)
                {
                    if ((*it)->tresProche(*successeur) && (*it)->distanceParcourue < successeur->distanceParcourue)
                    {
                        //cout << "abandon deja cherche" << endl;
                        abandon = true;
                        break;
                    }
                }
                if (!abandon)
                {
                    for (auto it = pointsAChercher.begin(); it != pointsAChercher.end(); ++it)
                    {
                        if ((*it)->tresProche(*successeur) && (*it)->distanceParcourue < successeur->distanceParcourue)
                        {
                            //cout << "abandon a chercher" << endl;
                            abandon = true;
                            break;
                        }
                    }
                    if (!abandon)
                    {
                        //cout << "ajout" << endl;
                        pointsAChercher.push_back(successeur);
                    }
                }
            }
        }
        pointsDejaCherches.push_back(candidat);
        iterations++;
    }
    cout << "limite atteinte " << endl;
    nettoyerListes(pointsAChercher, pointsDejaCherches);
    return false;
}

//Fonctions de test

void testRotation()
{
    double xr = 0;
    double yr = 0;
    rotation(0, 0, 0, 1, M_PI, xr, yr);
    cout << "testRotation : " << xr << ", " << yr << "  ok" << endl;
}

void testHeuristiqueInaccessible()
{
    PointAngle start = PointAngle(0, 0, 0);
    double epsilon = 0.0000000001;
    bool test1 = heuristiqueMeilleureDistanceInaccessible(start, PointAngle(0, 1, 0), 1.0) - (acos(7.0 / 8.0) + acos(1.0 / 4.0)) < epsilon;
    bool test2 = heuristiqueMeilleureDistanceInaccessible(start, PointAngle(0, 2, 0), 1.0) - (acos(-1.0) + acos(1.0)) < epsilon;
    ;
    bool test3 = heuristiqueMeilleureDistanceInaccessible(start, PointAngle(0, 0, 0), 1.0) - 0.0 < epsilon;

    cout << "quand l = 2 " << test1 << endl;
    cout << "quand l = 3 " << test2 << endl;
    cout << "quand l = 1 " << test3 << endl;
}

void testMeilleureManoeuvreSansAngleXYPreferer(double x, double y, bool prefererArriere)
{
    cout << "point a tester : " << x << ", " << y << (prefererArriere ? " arriere" : "") << endl;

    double rayonBraquage = 1.90;
    PointAngle depart = PointAngle(0, 0, 0);
    PointAngle arrivee = PointAngle(x, y, 0);
    double distance1, distance2, angleArrivee;
    MOUVEMENT mouvement1, mouvement2;
    chercherMeilleureManoeuvreSansAngle(depart, arrivee, prefererArriere, rayonBraquage, &mouvement1, &distance1, &mouvement2, &distance2, &angleArrivee);

    //Application du mouvement
    PointAngle *apresMouvement1 = depart.bouge(mouvement1, distance1, rayonBraquage);
    PointAngle *apresMouvement2 = apresMouvement1->bouge(mouvement2, distance2, rayonBraquage);
    double epsilon = 0.000001;

    if (abs(apresMouvement2->x - x) < epsilon && abs(apresMouvement2->y - y) < epsilon)
    {
        cout << "ok position" << endl;
    }
    else
    {
        cout << "FAIL position" << endl;
        cout << apresMouvement2->x << " vs. " << x << endl;
        cout << apresMouvement2->y << " vs. " << y << endl;
    }

    if (abs(apresMouvement2->angleRad - angleArrivee) < epsilon)
    {
        cout << "ok angle" << endl;
    }

    delete apresMouvement1;
    delete apresMouvement2;
}
void testMeilleureManoeuvreSansAngle()
{
    double tests[8][2] = {
        //accessibles
        {4, 1},
        {-4, 1},
        {4, -1},
        {-4, -1},
        //inaccessibles
        {0.1, 0.5},
        {-0.1, 0.5},
        {0.1, -0.5},
        {-0.1, -0.5}};
    for (unsigned int i = 0; i < 8; i++)
    {
        testMeilleureManoeuvreSansAngleXYPreferer(tests[i][0], tests[i][1], true);
        testMeilleureManoeuvreSansAngleXYPreferer(tests[i][0], tests[i][1], false);
    }
}

int main()
{
    //testRotation();
    //testMouvements();
    //testHeuristiqueInaccessible();

    //testMeilleureManoeuvreSansAngle();

    cout << " 1 : calculer manoeuvre avec angle (algo de recherche), 2 : calculer manoeuvre sans angle" << endl;
    int l = 2;
    cin >> l;
    cout << (l == 1 ? "manoeuvre avec angle (algo de recherche) !" : "manoeuvre sans angle" ) << endl;

    double x, y;
    cout << " x ? " << endl;
    cin >> x;
    cout << " y ? " << endl;
    cin >> y;

    if (l == 1)
    {
        double angle;
        cout << "angle en deg ? " << endl;
        cin >> angle;
        angle = angle * M_PI / 180.0;

        cout << "calcul en cours..." << endl;
        chercherMeilleureManoeuvre(PointAngle(x, y, angle));
    }
    else
    {
        bool prefererArriere = true;
        cout << " preferer en arriere ? 0/1" << endl;
        cin >> prefererArriere;
        double distance1, distance2, angleArrivee;
        MOUVEMENT mouvement1, mouvement2;
        PointAngle depart(0, 0, 0);
        chercherMeilleureManoeuvreSansAngle(depart, PointAngle(x, y, 0), prefererArriere, 1.90, &mouvement1, &distance1, &mouvement2, &distance2, &angleArrivee);
        cout << "mouvement 1 : " << MOUVEMENTS_TEXTE[mouvement1] << " avec distance " << distance1 << endl;
        cout << "mouvement 2 : " << MOUVEMENTS_TEXTE[mouvement2] << " avec distance " << distance2 << endl;
        cout << "arrivee avec un angle de " << (angleArrivee * 180.0 / M_PI) << " deg" <<endl;
    }

    return 0;
}
