#pragma once

#include <vector>
#include "lq2D.h"
#include "Item.h" 
#include <Eigen/Dense>
using namespace Eigen;
using std::vector; 

// ----------------------------------------------------------------------------
// A ProximityDatabase-style wrapper for the LQ bin lattice system
class LQProximityDatabase2D
{
public:

    // constructor
    LQProximityDatabase2D (const Vector2D& center,
                            const Vector2D& dimensions,
                            const Vector2D& divisions)
    {
        const Vector2D halfsize (dimensions * 0.5);
        const Vector2D origin (center - halfsize);
		_origin = origin;
		_dimensions = dimensions;
		_divisions = divisions;

        lq = lqCreateDatabase2D (origin.x(), origin.y(),
                                dimensions.x(), dimensions.y(),  
								(int) floor(0.5 + divisions.x()),
								(int) floor(0.5 + divisions.y()));
    }

    // destructor
    virtual ~LQProximityDatabase2D ()
    {
        lqDeleteDatabase2D (lq);
        lq = NULL;
    }

    // "token" to represent objects stored in the database
    class tokenType
    {
    public:

        // constructor
		tokenType (Item* parentObject, LQProximityDatabase2D& lqsd)
        {
            lqInitClientProxy2D (&proxy, parentObject);
            lq = lqsd.lq;
        }

        // destructor
        virtual ~tokenType (void)
        {
            lqRemoveFromBin (&proxy);
        }

        // the client object calls this each time its position changes
        void updateForNewPosition (const Vector2D& p)
        {
            lqUpdateForNewLocation (lq, &proxy, p.x(), p.y());
        }

        // find all neighbors within the given sphere (as center and radius)
        void findNeighbors (const Vector2D& center,
							const double radius,
                            vector<Item*>& results 
							)
        {
                lqMapOverAllObjectsInLocality (lq, center.x(), center.y(), 	radius, 
					[](void* clientObject, double distanceSquared, void* clientQueryState) 
					{
						vector<Item*>& results = *((vector<Item*>*) clientQueryState);
						results.push_back((Item*)clientObject); 
				    }, (void*)&results);
        }


    private:
        lqClientProxy2D proxy;
        lqInternalDB2D* lq;
    };

	// allocate a token to represent a given client object in this database
    tokenType* allocateToken (Item* item)
    {
        return new tokenType (item, *this);
    }

 	
	Vector2D getOrigin (void) {return _origin;}
	Vector2D getDivisions (void) {return _divisions;}
	Vector2D getDimensions (void) {return _dimensions;}

private:
    lqInternalDB2D* lq;
	Vector2D _origin;
	Vector2D _divisions;
	Vector2D _dimensions;
};

/// The spatial proximity database
typedef LQProximityDatabase2D SpatialProximityDatabase;

/// An object in the proximity database 
typedef SpatialProximityDatabase::tokenType ProximityToken;

