/*
*   This file is part of firstperson-telecontrol.
*
*    firstperson-telecontrol is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    firstperson-telecontrol is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with firstperson-telecontrol.  If not, see <http://www.gnu.org/licenses/>.
*
*	 Authors: Lars Fritsche, Felix Unverzagt, Roberto Calandra
*	 Created: July, 2015
*/


#define GLEW_STATIC
#include "GL/glew.h"
#define NO_STDIO_REDIRECT

#include <tchar.h>
#include <iostream>
#include <resource.h>
#include <ICubOculusController.h>
#include <OculusAdapter.h>
#include <YarpAdapter.h>
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include <chrono>
#include <fstream>

#include <OVR_CAPI.h>
#define OVR_OS_WIN32
#include "OVR_CAPI_GL.h"
#include "Kernel/OVR_Math.h"


#include <SDL.h>
#include <SDL_syswm.h>

using namespace OVR;
using namespace std;
using namespace std::chrono;

int fps = 200;
double spf = double(1) / double(fps);
int i = 0;

bool gazebo = true;

ovrHmd           HMD;                  // The handle of the headset
ovrEyeRenderDesc EyeRenderDesc[2];     // Description of the VR.
ovrRecti         EyeRenderViewport[2]; // Useful to remember when varying resolution
ovrPosef         EyeRenderPose[2];     // Useful to remember where the rendered eye originated

int WINAPI WinMain(HINSTANCE hinst, HINSTANCE, LPSTR, int) {

	SDL_Init(SDL_INIT_VIDEO);

	int x = SDL_WINDOWPOS_CENTERED;
	int y = SDL_WINDOWPOS_CENTERED;
	Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;

	bool debug = false;

	ovr_Initialize();

	ovrHmd hmd = ovrHmd_Create(0);

	if (hmd == NULL)
	{
		hmd = ovrHmd_CreateDebug(ovrHmd_DK2);

		debug = true;
	}

	if (debug == false && hmd->HmdCaps & ovrHmdCap_ExtendDesktop)
	{
		x = hmd->WindowsPos.x;
		y = hmd->WindowsPos.y;
		flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
	}

	YarpAdapter* yAdapter = new YarpAdapter();
	//YarpAdapter* yAdapter = NULL;
	OculusAdapter* oAdapter = new OculusAdapter(yAdapter);

	int w = hmd->Resolution.w;
	int h = hmd->Resolution.h;

	SDL_Window *window = SDL_CreateWindow("Oculus Rift SDL2 OpenGL Demo", x, y, w, h, flags);

	SDL_GLContext context = SDL_GL_CreateContext(window);

	glewExperimental = GL_TRUE;

	glewInit();

	Sizei recommendedTex0Size = ovrHmd_GetFovTextureSize(hmd, ovrEye_Left, hmd->DefaultEyeFov[0], 1.0f);
	Sizei recommendedTex1Size = ovrHmd_GetFovTextureSize(hmd, ovrEye_Right, hmd->DefaultEyeFov[1], 1.0f);
	Sizei renderTargetSize;
	renderTargetSize.w = recommendedTex0Size.w + recommendedTex1Size.w;
	renderTargetSize.h = max(recommendedTex0Size.h, recommendedTex1Size.h);

	GLuint frameBuffer;
	glGenFramebuffers(1, &frameBuffer);

	GLuint texture;
	glGenTextures(1, &texture);

	glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, renderTargetSize.w, renderTargetSize.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);

	GLuint renderBuffer;
	glGenRenderbuffers(1, &renderBuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, renderTargetSize.w, renderTargetSize.h);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderBuffer);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		glDeleteFramebuffers(1, &frameBuffer);
		glDeleteTextures(1, &texture);
		glDeleteRenderbuffers(1, &renderBuffer);

		SDL_GL_DeleteContext(context);

		SDL_DestroyWindow(window);

		ovrHmd_Destroy(hmd);

		ovr_Shutdown();

		SDL_Quit();

		return 0;
	}

	ovrFovPort eyeFov[2] = { hmd->DefaultEyeFov[0], hmd->DefaultEyeFov[1] };

	ovrRecti eyeRenderViewport[2];
	eyeRenderViewport[0].Pos = Vector2i(0, 0);
	eyeRenderViewport[0].Size = Sizei(renderTargetSize.w / 2, renderTargetSize.h);
	eyeRenderViewport[1].Pos = Vector2i((renderTargetSize.w + 1) / 2, 0);
	eyeRenderViewport[1].Size = eyeRenderViewport[0].Size;

	ovrGLTexture eyeTexture[2];
	eyeTexture[0].OGL.Header.API = ovrRenderAPI_OpenGL;
	eyeTexture[0].OGL.Header.TextureSize = renderTargetSize;
	eyeTexture[0].OGL.Header.RenderViewport = eyeRenderViewport[0];
	eyeTexture[0].OGL.TexId = texture;

	eyeTexture[1] = eyeTexture[0];
	eyeTexture[1].OGL.Header.RenderViewport = eyeRenderViewport[1];

	SDL_SysWMinfo info;

	SDL_VERSION(&info.version);

	SDL_GetWindowWMInfo(window, &info);

	ovrGLConfig cfg;
	cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
	cfg.OGL.Header.BackBufferSize = Sizei(hmd->Resolution.w, hmd->Resolution.h);
	cfg.OGL.Header.Multisample = 1;
#if defined(OVR_OS_WIN32)
	if (!(hmd->HmdCaps & ovrHmdCap_ExtendDesktop))
		ovrHmd_AttachToWindow(hmd, info.info.win.window, NULL, NULL);

	cfg.OGL.Window = info.info.win.window;
	cfg.OGL.DC = NULL;
#elif defined(OVR_OS_LINUX)
	cfg.OGL.Disp = info.info.x11.display;
	cfg.OGL.Win = info.info.x11.window;
#endif

	ovrEyeRenderDesc eyeRenderDesc[2];

	ovrHmd_ConfigureRendering(hmd, &cfg.Config, ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette | ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive, eyeFov, eyeRenderDesc);

	ovrHmd_SetEnabledCaps(hmd, ovrHmdCap_LowPersistence | ovrHmdCap_DynamicPrediction);

	ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);

	const GLchar *vertexShaderSource[] = {
		"#version 150\n"
		"uniform mat4 MVPMatrix;\n"
		"in vec3 position;\n"
		"in vec2 texcoord;\n"
		"out vec2 Texcoord;\n"
		"void main()\n"
		"{\n"
		"		Texcoord = texcoord;\n"
		"		gl_Position = MVPMatrix * vec4(position, 1.0);\n"
		"}"
	};

	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, vertexShaderSource, NULL);
	glCompileShader(vertexShader);

	const GLchar *fragmentShaderSource[] = {
		"#version 150\n"
		"in vec2 Texcoord;\n"
		"out vec4 outputColor;\n"
		"uniform sampler2D tex;\n"
		"void main()\n"
		"{\n"
		"    outputColor = texture(tex, Texcoord) * vec4(1.0, 1.0, 1.0, 1.0);\n"
		"}"
	};

	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);

	GLuint program = glCreateProgram();
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
	glLinkProgram(program);
	glUseProgram(program);

	GLuint MVPMatrixLocation = glGetUniformLocation(program, "MVPMatrix");
	GLuint positionLocation = glGetAttribLocation(program, "position");
	GLuint texLocation = glGetAttribLocation(program, "texcoord");

	GLuint vertexArray;
	glGenVertexArrays(1, &vertexArray);
	glBindVertexArray(vertexArray);

	GLfloat vertices[] = {
		-1.0f, -1.0f, -0.93f, 0.0f, 1.0f,
		-1.0f, 1.0f, -0.93f, 0.0f, 0.0f,
		1.0f, 1.0f, -0.93f, 1.0f, 0.0f,
		1.0f, -1.0f, -0.93f, 1.0f, 1.0f,
	};

	GLuint positionBuffer;
	glGenBuffers(1, &positionBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, positionBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(positionLocation, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), 0);
	glEnableVertexAttribArray(positionLocation);
	glEnableVertexAttribArray(texLocation);
	glVertexAttribPointer(texLocation, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);

	bool running = true;

	// bind pictures as texture
	GLuint imageTex;
	glGenTextures(1, &imageTex);
	glBindTexture(GL_TEXTURE_2D, imageTex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	int width = 320;
	int height = 240;

	int allPixels = width * height * 3;
	unsigned char * image = new unsigned char[allPixels];

	ovrHSWDisplayState hswDisplayState;
	ovrHmd_RecenterPose(hmd);
	bool initialized = false;
	//bool performMovement = false;
	while (running == true)
	{
		ovrHmd_GetHSWDisplayState(hmd, &hswDisplayState);
		if (hswDisplayState.Displayed) {
			ovrHmd_DismissHSWDisplay(hmd);
		}


		SDL_Event event;

		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
			case SDL_QUIT:
				running = false;
				break;
			case SDL_KEYDOWN:
				ovrHmd_DismissHSWDisplay(hmd);

				switch (event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					running = false;
					break;
				default:
					break;
				}
				break;
			case SDL_KEYUP:
				//performMovement = !performMovement;
				ovrHmd_RecenterPose(hmd);
				break;
			default:
				break;
			}
		}

		ovrHmd_BeginFrame(hmd, 0);

		ovrVector3f hmdToEyeViewOffset[2] = { eyeRenderDesc[0].HmdToEyeViewOffset, eyeRenderDesc[1].HmdToEyeViewOffset };

		ovrPosef eyeRenderPose[2];

		ovrHmd_GetEyePoses(hmd, 0, hmdToEyeViewOffset, eyeRenderPose, NULL);

		glBindFramebuffer(GL_FRAMEBUFFER, frameBuffer);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (!gazebo) {

			for (int eyeIndex = 0; eyeIndex < ovrEye_Count; eyeIndex++)
			{
				ovrEyeType eye = hmd->EyeRenderOrder[eyeIndex];

				if (!initialized) {
					Matrix4f MVPMatrix = Matrix4f(ovrMatrix4f_Projection(eyeRenderDesc[eye].Fov, 0.01f, 10000.0f, true)) * Matrix4f(Quatf(eyeRenderPose[eye].Orientation).Inverted()) * Matrix4f::Translation(-Vector3f(eyeRenderPose[eye].Position));
					glUniformMatrix4fv(MVPMatrixLocation, 1, GL_FALSE, &MVPMatrix.Transposed().M[0][0]);
					initialized = true;
				}

				glViewport(eyeRenderViewport[eye].Pos.x, eyeRenderViewport[eye].Pos.y, eyeRenderViewport[eye].Size.w, eyeRenderViewport[eye].Size.h);

				if (eyeIndex == 1)
					image = yAdapter->getImage(YarpAdapter::CAMERA_LEFT);
				else
					image = yAdapter->getImage(YarpAdapter::CAMERA_LEFT);
			

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);

			glDrawArrays(GL_QUADS, 0, 4);
			}
		}

		ovrVector3f useHmdToEyeViewOffset[2] = {
			EyeRenderDesc[0].HmdToEyeViewOffset,
			EyeRenderDesc[1].HmdToEyeViewOffset };


		ovrPosef temp_EyeRenderPose[2];
		ovrHmd_GetEyePoses(hmd, 0, useHmdToEyeViewOffset, temp_EyeRenderPose, NULL);

		double newVal[] = {
			(double)temp_EyeRenderPose[0].Orientation.x,
			(double)-temp_EyeRenderPose[0].Orientation.z,
			(double)temp_EyeRenderPose[0].Orientation.y };

		//vector<double*> angles = oAdapter->transformToAnglesInDegree(newVal);

		yAdapter->writeToOutput(6, newVal);
		//if (performMovement) 
		//	yAdapter->move(YarpAdapter::MOVE_HEAD, angles[0]);

		glBindVertexArray(0);

		ovrHmd_EndFrame(hmd, eyeRenderPose, &eyeTexture[0].Texture);

		glBindVertexArray(vertexArray);
			
		Sleep(spf * 1000);
	}

	glDeleteVertexArrays(1, &vertexArray);
	glDeleteBuffers(1, &positionBuffer);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	glDeleteProgram(program);

	glDeleteFramebuffers(1, &frameBuffer);
	glDeleteTextures(1, &texture);
	glDeleteRenderbuffers(1, &renderBuffer);

	SDL_GL_DeleteContext(context);

	SDL_DestroyWindow(window);

	ovrHmd_Destroy(hmd);

	ovr_Shutdown();

	SDL_Quit();

	return 0;
}