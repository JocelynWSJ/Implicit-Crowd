#pragma once
/**
	* @brief The virtual interface used by objects in the spatial database.
	*
*/
class Item {
public:
	/// Destructor.
	virtual ~Item() {}
	/// Returns true if the object is an agent, false if not.
	virtual bool isAgent() = 0;
		
};

/// Pointer to a ProximityDatabaseItem
typedef Item* ProximityDatabaseItemPtr;



