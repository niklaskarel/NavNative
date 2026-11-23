#include "ViewerWindow.h"
#include "nav/Logger/Log.h"
#include "nav/Routing/OpenStreetMap.h"

#include <QApplication>
#include <QMessageBox>

#include <exception>
#include <filesystem>
#include <fstream>
#include <utility>

// Loads the bundled map and runs the Qt event loop.
int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	try
	{
		const auto directory = std::filesystem::path(QApplication::applicationDirPath().toStdU16String());
		std::ifstream mapFile(directory / "Eixample.osm", std::ios::binary);
		if (!mapFile)
		{
			nav::logger::error("Could not open the bundled street map.");
			QMessageBox::critical(nullptr, "Could not start NavNative Lab", "Could not open the bundled street map.");
			return 1;
		}
		auto map = nav::routing::loadStreetMapFromOsmXml(mapFile);
		if (!map)
		{
			QMessageBox::critical(nullptr,
								  "Could not start NavNative Lab",
								  "Could not load the bundled street map. See the console log for details.");
			return 1;
		}
		nav::viewer::ViewerWindow window(std::move(*map));
		window.show();
		return app.exec();
	}
	catch (const std::exception& error)
	{
		nav::logger::error(error.what());
		QMessageBox::critical(nullptr, "Could not start NavNative Lab", QString::fromUtf8(error.what()));
		return 1;
	}
}
