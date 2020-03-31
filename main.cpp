#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <list>

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
    vector<pair<MOUVEMENT, double>> mouvements;

    PointAngle() : x(0), y(0), angleRad(0), distanceParcourue(0), estimation(0), mouvements(vector<pair<MOUVEMENT, double>>())
    {
    }
    PointAngle(double _x, double _y, double _angleRad) : x(_x), y(_y), angleRad(_angleRad), distanceParcourue(0), estimation(0), mouvements(vector<pair<MOUVEMENT, double>>())
    {
    }

    PointAngle(double _x, double _y, double _angleRad,
               double _distanceParcouruePere, MOUVEMENT mouvement, vector<pair<MOUVEMENT, double>> &mouvementsPere, double pas)
        : x(_x), angleRad(0.f), y(_y), distanceParcourue(_distanceParcouruePere + pas), estimation(0.f), mouvements(vector<pair<MOUVEMENT, double>>())
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
        //On recopie toutes les valeurs...
        //Si la derniere valeur = mouvement, on additione
        //Sinon, on rajoute
        for (int i = 0; i < mouvementsPere.size(); i++)
        {
            mouvements.push_back(mouvementsPere[i]);
        }

        if (mouvementsPere.size() != 0 && mouvementsPere.back().first == mouvement)
        {
            mouvements.back() = make_pair(mouvements.back().first, mouvements.back().second + pas);
        }
        else
        {
            mouvements.push_back(make_pair(mouvement, pas));
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
        return new PointAngle(xFils, yFils, angleRad, distanceParcourue, AVANCE, mouvements, pas);
    }
    PointAngle *recule(double pas)
    {
        double xr, yr;
        vecteurAngle(xr, yr);
        double xFils = x - xr * pas;
        double yFils = y - yr * pas;
        return new PointAngle(xFils, yFils, angleRad, distanceParcourue, RECULE, mouvements, pas);
    }
    PointAngle *avantDroit(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = -pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleDroite(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue, AVANT_DROIT, mouvements, pas);
    }
    PointAngle *avantGauche(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleGauche(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue, AVANT_GAUCHE, mouvements, pas);
    }
    PointAngle *arriereDroit(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleDroite(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue, ARRIERE_DROIT, mouvements, pas);
    }
    PointAngle *arriereGauche(double pas, double rayonBraquage)
    {
        double xr, yr, angleRotation = -pas / rayonBraquage; // vers ma droite => sens horaire => anti sens trigo
        vecteurAngleGauche(xr, yr);
        double xCentre = x + rayonBraquage * xr;
        double yCentre = y + rayonBraquage * yr;
        double xPos, yPos;
        rotation(x, y, xCentre, yCentre, angleRotation, xPos, yPos);
        return new PointAngle(xPos, yPos, angleRad + angleRotation, distanceParcourue, ARRIERE_GAUCHE, mouvements, pas);
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

    string manoeuvresString()
    {
        string res = "[";
        int size =  mouvements.size();
        for (auto i = 0; i < size; i++)
        {
            ostringstream ss;
            ss << mouvements[i].second;
            string s(ss.str());

            res += "('" + MOUVEMENTS_TEXTE[mouvements[i].first] + "', " + s + ")";
            if (i != size - 1)
            {
                res += ", ";
            }
        }

        res += "]";
        return res;
    }
};

ostream &operator<<(ostream &strm, const PointAngle &pa)
{
    return strm << "PointAngle(x = " << pa.x << ", y = " << pa.y << ", angle = " << (pa.angleRad * 180.0 / M_PI)
                << ", dist = " << pa.distanceParcourue << " )";
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
        cout << *resultat << " manoeuvres : " << resultat->manoeuvresString() << endl;
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
double heuristique(const PointAngle &pointAngle, const PointAngle &butPointAngle, const double rayonBraquage)
{
    return max(distance(pointAngle, butPointAngle), heuristiqueAngle(pointAngle, butPointAngle, rayonBraquage));
}

PointAngle *prendreMeilleurCandidat(list<PointAngle *> &pointsAChercher, const PointAngle &butPointAngle, double rayonBraquage)
{
    double meilleureEstimation = 100000000, estimation = 0;
    list<PointAngle *>::iterator meilleurCandidatIterator = pointsAChercher.begin();
    int i = 0;
    for (auto pointIterator = pointsAChercher.begin(); pointIterator != pointsAChercher.end(); pointIterator++)
    {
        estimation = 0;
        if ((*pointIterator)->estimation != 0) {
            estimation = (*pointIterator)->estimation;
        }
        else
        {
            (*pointIterator)->estimation = estimation = (*pointIterator)->distanceParcourue + heuristique(**pointIterator, butPointAngle, rayonBraquage);
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
        PointAngle *candidat = prendreMeilleurCandidat(pointsAChercher, butPointAngle, rayonBraquage);
        cout << *candidat << endl;

        
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
                cout << successeur->manoeuvresString() << endl;
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

void testRotation()
{
    double xr = 0;
    double yr = 0;
    rotation(0, 0, 0, 1, M_PI, xr, yr);
    cout << "hello gateau : " << xr << ", " << yr << "  ok" << endl;
}

int main()
{
    //testRotation();
    //testMouvements();
    PointAngle but(0.0, 0.7, 0.0);

    chercherMeilleureManoeuvre(but);

    double wait = 0.f;
    cin >> wait;

    return 0;
}
