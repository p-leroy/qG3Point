#include "ActionA.h"

#include "ccMainAppInterface.h"
#include "ccPointCloud.h"

namespace G3Point
{

	void find_neighbors()
{
	}

	void performActionA( ccMainAppInterface *appInterface )
	{
		if ( appInterface == nullptr )
		{
			// The application interface should have already been initialized when the plugin is loaded
			Q_ASSERT( false );
			
			return;
		}
		
		/*** HERE STARTS THE ACTION ***/

		// Find neighbors of each point of the cloud


		// Perform initial segmentation

		// This is how you can output messages
		// Display a standard message in the console
		appInterface->dispToConsole( "[ExamplePlugin] Hello world!", ccMainAppInterface::STD_CONSOLE_MESSAGE );
		
		// Display a warning message in the console
		appInterface->dispToConsole( "[ExamplePlugin] Warning: example plugin shouldn't be used as is", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
		
		// Display an error message in the console AND pop-up an error box
		appInterface->dispToConsole( "Example plugin shouldn't be used - it doesn't do anything!", ccMainAppInterface::ERR_CONSOLE_MESSAGE );
	
		/*** HERE ENDS THE ACTION ***/
	}
}
