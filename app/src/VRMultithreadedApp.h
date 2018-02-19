#ifndef VRMULTITHREADEDAPP_H_
#define VRMULTITHREADEDAPP_H_

#include <main/VRMain.h>
#include <api/VRGraphicsState.h>
#include <map>
#include <mutex>

namespace MinVR {

class VRAppSharedContext {
public:
	VRAppSharedContext() : version(0) {}
	virtual ~VRAppSharedContext() {}

	void update(const VRGraphicsState &renderState, int version);

protected:
	virtual void update(const VRGraphicsState &renderState) {}

private:
	std::mutex mutex;
	int version;
};

class VRAppRenderer {
public:
	virtual ~VRAppRenderer() {}

	virtual void update(const VRGraphicsState &renderState) {}
	virtual void render(const VRGraphicsState &renderState) {}
};

class VRMultithreadedApp :  public VREventHandler, public VRRenderHandler {
public:
	VRMultithreadedApp(int argc, char** argv);
	virtual ~VRMultithreadedApp();

	/// Handles and updates Graphics Context
	//void onVRRenderGraphicsContext(const MinVR::VRGraphicsState &renderState);
	/// Handles Graphics rendering
	//void onVRRenderGraphics(const MinVR::VRGraphicsState &renderState);

	virtual void onVRRenderContext(const VRDataIndex &stateData);
	virtual void onVRRenderScene(const VRDataIndex &stateData);

	virtual VRAppSharedContext* createSharedContext(const VRGraphicsState &renderState) { return new VRAppSharedContext(); }
	virtual VRAppRenderer* createRenderer(VRAppSharedContext& sharedContext, const VRGraphicsState &renderState) = 0;

	virtual void update() {}

	void run();

	bool isRunning() { return running; }
	void shutdown() { running = false; }

protected:
	std::map<int, VRAppRenderer*> views;
	std::map<int, VRAppSharedContext*> sharedContexts;
	std::map<int, int> sharedContextCount;
	std::map<int, VRAppSharedContext*> normalContexts;
	std::mutex mutex;
	int frame;
	VRMain* vrmain;
	bool running;
};

//--------------------- Implementation -------------------------

inline void VRAppSharedContext::update(const VRGraphicsState& renderState, int version) {
	// In this application, only one window will update the shared context at a time, hence the locking
	if (version != this->version) {
		std::unique_lock<std::mutex> lock(mutex);
		if (version != this->version) {

			update(renderState);

			this->version = version;
		}
	}
}

inline VRMultithreadedApp::VRMultithreadedApp(int argc, char** argv) : running(false), frame(0) {
    vrmain = new VRMain();
    vrmain->addEventHandler(this);
    vrmain->addRenderHandler(this);
    vrmain->initialize(argc,argv);
}

inline VRMultithreadedApp::~VRMultithreadedApp() {
	vrmain->shutdown();
	delete vrmain;
}

inline void VRMultithreadedApp::run() {
	running = true;
	while (running) {
		vrmain->synchronizeAndProcessEvents();
		update();
		vrmain->renderOnAllDisplays();
		frame++;
	}
}

inline void VRMultithreadedApp::onVRRenderContext(const VRDataIndex &stateData) {
	if (stateData.exists("IsGraphics")) {
        VRGraphicsState renderState(stateData);
        int windowId = renderState.getWindowId();
		int sharedContextId = renderState.getSharedContextId();

		// If this is the inital call, initialize context variables
		if (renderState.isInitialRenderCall()) {
			// Need to lock here so that we can add the context objects one by one
			std::unique_lock<std::mutex> lock(mutex);

			// If a shared context Id exists, create one per shared context, otherwise one per window
			VRAppSharedContext* context = NULL;
			if (sharedContextId < 0) {
				// Create a shared context per window if this window doesn't have a shared context
				context = createSharedContext(renderState);
				normalContexts[windowId] = context;
			}
			else {
				// Use shared context if it already exists, otherwise create one
				std::map<int, VRAppSharedContext*>::iterator it = sharedContexts.find(sharedContextId);
				if (it == sharedContexts.end()) {
					context = createSharedContext(renderState);
					sharedContexts[sharedContextId] = context;
					sharedContextCount[sharedContextId] = 1;
				}
				else {
					context = it->second;
					sharedContextCount[sharedContextId]++;
				}
			}

			// Create one view per window and use shared context
			views[windowId] = createRenderer(*context, renderState);
			lock.unlock();
		}
		else {
			if (sharedContextId < 0) {
				// Update window shared context if no shared context exists
				normalContexts[windowId]->update(renderState, frame);
			}
			else {
				// Update shared context if it exists
				sharedContexts[sharedContextId]->update(renderState, frame);
			}

			// Update the view for the window
			views[windowId]->update(renderState);
		}

		// Destroy context items if the program is no longer running
		if (!isRunning()) {
			// Need to lock for deleting
			std::unique_lock<std::mutex> lock(mutex);

			// Delete view
			{
				std::map<int, VRAppRenderer*>::iterator it = views.find(windowId);
				delete it->second;
				views.erase(it);
			}

			// Delete shared contexts
			if (sharedContextId < 0) {
				std::map<int, VRAppSharedContext*>::iterator it = normalContexts.find(windowId);
				delete it->second;
				normalContexts.erase(it);
			}
			else {
				sharedContextCount[sharedContextId]--;
				if (sharedContextCount[sharedContextId] == 0) {
					std::map<int, VRAppSharedContext*>::iterator it = sharedContexts.find(sharedContextId);
					delete it->second;
					sharedContexts.erase(it);
				}
			}

			lock.unlock();

			return;
		}
    }
}

/// onVRRenderScene will run draw calls on each viewport inside a context.
inline void VRMultithreadedApp::onVRRenderScene(const VRDataIndex &stateData) {
	if (stateData.exists("IsGraphics")) {
        VRGraphicsState renderState(stateData);
		// Only draw if the application is still running.
		if (isRunning()) {
			int windowId = renderState.getWindowId();

			// Render on view
			views[windowId]->render(renderState);
		}
	}
}

}

#endif