#pragma once

#ifndef QUANERGY_CLIENT_DLL_EXPORT_H
#define QUANERGY_CLIENT_DLL_EXPORT_H

#ifdef _MSC_VER
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

#endif // !QUANERGY_CLIENT_DLL_EXPORT
