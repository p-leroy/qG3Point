//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: G3PointPlugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#include <QtGui>

#include "G3Point.h"

#include "G3PointAction.h"

ccHObject* G3PointFactory::buildObject(const QString& metaName)
{
	if (metaName == "GrainsAsEllipsoids")
	{
		ccLog::Warning("[G3PointFactory::buildObject] build a GrainsAsEllipsoids object");
		return new GrainsAsEllipsoids(m_app);
	}
	else
	{
		ccLog::Warning("[G3PointFactory::buildObject] you are asking for the building of an unknown object: " + metaName);
		return nullptr;
	}
}

// Default constructor:
//	- pass the Qt resource path to the info.json file (from <yourPluginName>.qrc file) 
//  - constructor should mainly be used to initialize actions and other members
G3PointPlugin::G3PointPlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/G3PointPlugin/info.json" )
	, m_action( nullptr )
{
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void G3PointPlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	// If you need to check for a specific type of object, you can use the methods
	// in ccHObjectCaster.h or loop and check the objects' classIDs like this:
	//
	//	for ( ccHObject *object : selectedEntities )
	//	{
	//		if ( object->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
	//		{
	//			// ... do something with the viewports
	//		}
	//	}
	
	// For example - only enable our action if something is selected.
	m_action->setEnabled( !selectedEntities.empty() );
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> G3PointPlugin::getActions()
{
	ccExternalFactory::Container::Shared container = ccExternalFactory::Container::GetUniqueInstance();
	G3PointFactory* g3PointFactory = new G3PointFactory("G3Point", getMainAppInterface());
	container->addFactory(g3PointFactory);

	// default action (if it has not been already created, this is the moment to do it)
	if ( !m_action )
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, [this]()
		{
			G3Point::G3PointAction::createAction(m_app);
		});
	}

	return { m_action };
}
