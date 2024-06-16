﻿using System.Reflection;
using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete
{
    internal class GeozillaCoreDll
    {
#if DEBUG
        private const string ConfigName = "Debug";
#else
        private const string ConfigName = "Release";
#endif

        private const string DllName = "geozilla-core.dll";

        [DllImport("kernel32.dll")]
        public static extern IntPtr LoadLibrary(string dllToLoad);

        [DllImport(DllName, EntryPoint = "?GenerateGeoJson@@YAPEBDPEBD@Z", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GenerateGeoJson([MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport(DllName, EntryPoint = "?FreeBuffer@@YAXPEBD@Z", CallingConvention = CallingConvention.Cdecl)]
        public static extern void FreeBuffer(IntPtr buffer);

        static GeozillaCoreDll()
        {
            string applicationDirectory = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location)!;
            string dllPath = Path.Combine(applicationDirectory!, "..", "..", "..", "..", "..", "geozilla-core", "bin", ConfigName, DllName);
            LoadLibrary(Path.GetFullPath(dllPath));
        }
    }
}
