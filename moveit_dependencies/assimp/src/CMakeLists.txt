set(ASSIMP_LOADER_SRCS "")

MACRO(ADD_ASSIMP_IMPORTER name)
    list(APPEND ASSIMP_LOADER_SRCS ${ARGN})
ENDMACRO()

ADD_ASSIMP_IMPORTER(AMF
        AssetLib/AMF/AMFImporter.hpp
        AssetLib/AMF/AMFImporter_Node.hpp
        AssetLib/AMF/AMFImporter.cpp
        AssetLib/AMF/AMFImporter_Geometry.cpp
        AssetLib/AMF/AMFImporter_Material.cpp
        AssetLib/AMF/AMFImporter_Postprocess.cpp
        )

ADD_ASSIMP_IMPORTER(3DS
        AssetLib/3DS/3DSConverter.cpp
        AssetLib/3DS/3DSHelper.h
        AssetLib/3DS/3DSLoader.cpp
        AssetLib/3DS/3DSLoader.h
        )

ADD_ASSIMP_IMPORTER(AC
        AssetLib/AC/ACLoader.cpp
        AssetLib/AC/ACLoader.h
        )

ADD_ASSIMP_IMPORTER(ASE
        AssetLib/ASE/ASELoader.cpp
        AssetLib/ASE/ASELoader.h
        AssetLib/ASE/ASEParser.cpp
        AssetLib/ASE/ASEParser.h
        )

ADD_ASSIMP_IMPORTER(ASSBIN
        AssetLib/Assbin/AssbinLoader.h
        AssetLib/Assbin/AssbinLoader.cpp
        )

ADD_ASSIMP_IMPORTER(B3D
        AssetLib/B3D/B3DImporter.cpp
        AssetLib/B3D/B3DImporter.h
        )

ADD_ASSIMP_IMPORTER(BVH
        AssetLib/BVH/BVHLoader.cpp
        AssetLib/BVH/BVHLoader.h
        )

ADD_ASSIMP_IMPORTER(COLLADA
        AssetLib/Collada/ColladaHelper.cpp
        AssetLib/Collada/ColladaHelper.h
        AssetLib/Collada/ColladaLoader.cpp
        AssetLib/Collada/ColladaLoader.h
        AssetLib/Collada/ColladaParser.cpp
        AssetLib/Collada/ColladaParser.h
        )

ADD_ASSIMP_IMPORTER(DXF
        AssetLib/DXF/DXFLoader.cpp
        AssetLib/DXF/DXFLoader.h
        AssetLib/DXF/DXFHelper.h
        )

ADD_ASSIMP_IMPORTER(CSM
        AssetLib/CSM/CSMLoader.cpp
        AssetLib/CSM/CSMLoader.h
        )

ADD_ASSIMP_IMPORTER(HMP
        AssetLib/HMP/HMPFileData.h
        AssetLib/HMP/HMPLoader.cpp
        AssetLib/HMP/HMPLoader.h
        AssetLib/HMP/HalfLifeFileData.h
        )

ADD_ASSIMP_IMPORTER(IRRMESH
        AssetLib/Irr/IRRMeshLoader.cpp
        AssetLib/Irr/IRRMeshLoader.h
        AssetLib/Irr/IRRShared.cpp
        AssetLib/Irr/IRRShared.h
        )

ADD_ASSIMP_IMPORTER(IRR
        AssetLib/Irr/IRRLoader.cpp
        AssetLib/Irr/IRRLoader.h
        AssetLib/Irr/IRRShared.cpp
        AssetLib/Irr/IRRShared.h
        )

ADD_ASSIMP_IMPORTER(LWO
        AssetLib/LWO/LWOAnimation.cpp
        AssetLib/LWO/LWOAnimation.h
        AssetLib/LWO/LWOBLoader.cpp
        AssetLib/LWO/LWOFileData.h
        AssetLib/LWO/LWOLoader.cpp
        AssetLib/LWO/LWOLoader.h
        AssetLib/LWO/LWOMaterial.cpp
        )

ADD_ASSIMP_IMPORTER(LWS
        AssetLib/LWS/LWSLoader.cpp
        AssetLib/LWS/LWSLoader.h
        )

ADD_ASSIMP_IMPORTER(M3D
        AssetLib/M3D/M3DMaterials.h
        AssetLib/M3D/M3DImporter.h
        AssetLib/M3D/M3DImporter.cpp
        AssetLib/M3D/M3DWrapper.h
        AssetLib/M3D/M3DWrapper.cpp
        AssetLib/M3D/m3d.h
        )

ADD_ASSIMP_IMPORTER(MD2
        AssetLib/MD2/MD2FileData.h
        AssetLib/MD2/MD2Loader.cpp
        AssetLib/MD2/MD2Loader.h
        AssetLib/MD2/MD2NormalTable.h
        )

ADD_ASSIMP_IMPORTER(MD3
        AssetLib/MD3/MD3FileData.h
        AssetLib/MD3/MD3Loader.cpp
        AssetLib/MD3/MD3Loader.h
        )

ADD_ASSIMP_IMPORTER(MD5
        AssetLib/MD5/MD5Loader.cpp
        AssetLib/MD5/MD5Loader.h
        AssetLib/MD5/MD5Parser.cpp
        AssetLib/MD5/MD5Parser.h
        )

ADD_ASSIMP_IMPORTER(MDC
        AssetLib/MDC/MDCFileData.h
        AssetLib/MDC/MDCLoader.cpp
        AssetLib/MDC/MDCLoader.h
        AssetLib/MDC/MDCNormalTable.h
        )

ADD_ASSIMP_IMPORTER(MDL
        AssetLib/MDL/MDLDefaultColorMap.h
        AssetLib/MDL/MDLFileData.h
        AssetLib/MDL/MDLLoader.cpp
        AssetLib/MDL/MDLLoader.h
        AssetLib/MDL/MDLMaterialLoader.cpp
        AssetLib/MDL/HalfLife/HalfLifeMDLBaseHeader.h
        AssetLib/MDL/HalfLife/HL1FileData.h
        AssetLib/MDL/HalfLife/HL1MDLLoader.cpp
        AssetLib/MDL/HalfLife/HL1MDLLoader.h
        AssetLib/MDL/HalfLife/HL1ImportDefinitions.h
        AssetLib/MDL/HalfLife/HL1ImportSettings.h
        AssetLib/MDL/HalfLife/HL1MeshTrivert.h
        AssetLib/MDL/HalfLife/LogFunctions.h
        AssetLib/MDL/HalfLife/UniqueNameGenerator.cpp
        AssetLib/MDL/HalfLife/UniqueNameGenerator.h
        )

ADD_ASSIMP_IMPORTER(NFF
        AssetLib/NFF/NFFLoader.cpp
        AssetLib/NFF/NFFLoader.h
        )

ADD_ASSIMP_IMPORTER(NDO
        AssetLib/NDO/NDOLoader.cpp
        AssetLib/NDO/NDOLoader.h
        )

ADD_ASSIMP_IMPORTER(OFF
        AssetLib/OFF/OFFLoader.cpp
        AssetLib/OFF/OFFLoader.h
        )

ADD_ASSIMP_IMPORTER(OBJ
        AssetLib/Obj/ObjFileData.h
        AssetLib/Obj/ObjFileImporter.cpp
        AssetLib/Obj/ObjFileImporter.h
        AssetLib/Obj/ObjFileMtlImporter.cpp
        AssetLib/Obj/ObjFileMtlImporter.h
        AssetLib/Obj/ObjFileParser.cpp
        AssetLib/Obj/ObjFileParser.h
        AssetLib/Obj/ObjTools.h
        )

ADD_ASSIMP_IMPORTER(OGRE
        AssetLib/Ogre/OgreImporter.h
        AssetLib/Ogre/OgreStructs.h
        AssetLib/Ogre/OgreParsingUtils.h
        AssetLib/Ogre/OgreBinarySerializer.h
        AssetLib/Ogre/OgreXmlSerializer.h
        AssetLib/Ogre/OgreImporter.cpp
        AssetLib/Ogre/OgreStructs.cpp
        AssetLib/Ogre/OgreBinarySerializer.cpp
        AssetLib/Ogre/OgreXmlSerializer.cpp
        AssetLib/Ogre/OgreMaterial.cpp
        )

ADD_ASSIMP_IMPORTER(OPENGEX
        AssetLib/OpenGEX/OpenGEXImporter.cpp
        AssetLib/OpenGEX/OpenGEXImporter.h
        AssetLib/OpenGEX/OpenGEXStructs.h
        )

ADD_ASSIMP_IMPORTER(PLY
        AssetLib/Ply/PlyLoader.cpp
        AssetLib/Ply/PlyLoader.h
        AssetLib/Ply/PlyParser.cpp
        AssetLib/Ply/PlyParser.h
        )

ADD_ASSIMP_IMPORTER(MS3D
        AssetLib/MS3D/MS3DLoader.cpp
        AssetLib/MS3D/MS3DLoader.h
        )

ADD_ASSIMP_IMPORTER(COB
        AssetLib/COB/COBLoader.cpp
        AssetLib/COB/COBLoader.h
        AssetLib/COB/COBScene.h
        )

ADD_ASSIMP_IMPORTER(BLEND
        AssetLib/Blender/BlenderLoader.cpp
        AssetLib/Blender/BlenderLoader.h
        AssetLib/Blender/BlenderDNA.cpp
        AssetLib/Blender/BlenderDNA.h
        AssetLib/Blender/BlenderDNA.inl
        AssetLib/Blender/BlenderScene.cpp
        AssetLib/Blender/BlenderScene.h
        AssetLib/Blender/BlenderSceneGen.h
        AssetLib/Blender/BlenderIntermediate.h
        AssetLib/Blender/BlenderModifier.h
        AssetLib/Blender/BlenderModifier.cpp
        AssetLib/Blender/BlenderBMesh.h
        AssetLib/Blender/BlenderBMesh.cpp
        AssetLib/Blender/BlenderTessellator.h
        AssetLib/Blender/BlenderTessellator.cpp
        AssetLib/Blender/BlenderCustomData.h
        AssetLib/Blender/BlenderCustomData.cpp
        )

ADD_ASSIMP_IMPORTER(IFC
        AssetLib/IFC/IFCLoader.cpp
        AssetLib/IFC/IFCLoader.h
        AssetLib/IFC/IFCReaderGen1_2x3.cpp
        AssetLib/IFC/IFCReaderGen2_2x3.cpp
        AssetLib/IFC/IFCReaderGen_2x3.h
        AssetLib/IFC/IFCUtil.h
        AssetLib/IFC/IFCUtil.cpp
        AssetLib/IFC/IFCGeometry.cpp
        AssetLib/IFC/IFCMaterial.cpp
        AssetLib/IFC/IFCProfile.cpp
        AssetLib/IFC/IFCCurve.cpp
        AssetLib/IFC/IFCBoolean.cpp
        AssetLib/IFC/IFCOpenings.cpp
        )

ADD_ASSIMP_IMPORTER(XGL
        AssetLib/XGL/XGLLoader.cpp
        AssetLib/XGL/XGLLoader.h
        )

ADD_ASSIMP_IMPORTER(FBX
        AssetLib/FBX/FBXImporter.cpp
        AssetLib/FBX/FBXCompileConfig.h
        AssetLib/FBX/FBXImporter.h
        AssetLib/FBX/FBXParser.cpp
        AssetLib/FBX/FBXParser.h
        AssetLib/FBX/FBXTokenizer.cpp
        AssetLib/FBX/FBXTokenizer.h
        AssetLib/FBX/FBXImportSettings.h
        AssetLib/FBX/FBXConverter.h
        AssetLib/FBX/FBXConverter.cpp
        AssetLib/FBX/FBXUtil.h
        AssetLib/FBX/FBXUtil.cpp
        AssetLib/FBX/FBXDocument.h
        AssetLib/FBX/FBXDocument.cpp
        AssetLib/FBX/FBXProperties.h
        AssetLib/FBX/FBXProperties.cpp
        AssetLib/FBX/FBXMeshGeometry.h
        AssetLib/FBX/FBXMeshGeometry.cpp
        AssetLib/FBX/FBXMaterial.cpp
        AssetLib/FBX/FBXModel.cpp
        AssetLib/FBX/FBXAnimation.cpp
        AssetLib/FBX/FBXNodeAttribute.cpp
        AssetLib/FBX/FBXDeformer.cpp
        AssetLib/FBX/FBXBinaryTokenizer.cpp
        AssetLib/FBX/FBXDocumentUtil.cpp
        AssetLib/FBX/FBXCommon.h
        )

ADD_ASSIMP_IMPORTER(Q3D
        AssetLib/Q3D/Q3DLoader.cpp
        AssetLib/Q3D/Q3DLoader.h
        )

ADD_ASSIMP_IMPORTER(Q3BSP
        AssetLib/Q3BSP/Q3BSPFileData.h
        AssetLib/Q3BSP/Q3BSPFileParser.h
        AssetLib/Q3BSP/Q3BSPFileParser.cpp
        AssetLib/Q3BSP/Q3BSPFileImporter.h
        AssetLib/Q3BSP/Q3BSPFileImporter.cpp
        )

ADD_ASSIMP_IMPORTER(RAW
        AssetLib/Raw/RawLoader.cpp
        AssetLib/Raw/RawLoader.h
        )

ADD_ASSIMP_IMPORTER(SIB
        AssetLib/SIB/SIBImporter.cpp
        AssetLib/SIB/SIBImporter.h
        )

ADD_ASSIMP_IMPORTER(SMD
        AssetLib/SMD/SMDLoader.cpp
        AssetLib/SMD/SMDLoader.h
        )

ADD_ASSIMP_IMPORTER(STL
        AssetLib/STL/STLLoader.cpp
        AssetLib/STL/STLLoader.h
        )

ADD_ASSIMP_IMPORTER(TERRAGEN
        AssetLib/Terragen/TerragenLoader.cpp
        AssetLib/Terragen/TerragenLoader.h
        )

ADD_ASSIMP_IMPORTER(3D
        AssetLib/Unreal/UnrealLoader.cpp
        AssetLib/Unreal/UnrealLoader.h
        )

ADD_ASSIMP_IMPORTER(X
        AssetLib/X/XFileHelper.h
        AssetLib/X/XFileImporter.cpp
        AssetLib/X/XFileImporter.h
        AssetLib/X/XFileParser.cpp
        AssetLib/X/XFileParser.h
        )

ADD_ASSIMP_IMPORTER(X3D
        AssetLib/X3D/X3DImporter.cpp
        AssetLib/X3D/X3DImporter.hpp
        )

ADD_ASSIMP_IMPORTER(GLTF
        AssetLib/glTF/glTFCommon.h
        AssetLib/glTF/glTFCommon.cpp
        AssetLib/glTF/glTFAsset.h
        AssetLib/glTF/glTFAsset.inl
        AssetLib/glTF/glTFAssetWriter.h
        AssetLib/glTF/glTFAssetWriter.inl
        AssetLib/glTF/glTFImporter.cpp
        AssetLib/glTF/glTFImporter.h
        AssetLib/glTF2/glTF2Asset.h
        AssetLib/glTF2/glTF2Asset.inl
        AssetLib/glTF2/glTF2AssetWriter.h
        AssetLib/glTF2/glTF2AssetWriter.inl
        AssetLib/glTF2/glTF2Importer.cpp
        AssetLib/glTF2/glTF2Importer.h
        )

ADD_ASSIMP_IMPORTER(3MF
        AssetLib/3MF/D3MFImporter.h
        AssetLib/3MF/D3MFImporter.cpp
        AssetLib/3MF/D3MFOpcPackage.h
        AssetLib/3MF/D3MFOpcPackage.cpp
        AssetLib/3MF/3MFXmlTags.h
        )

ADD_ASSIMP_IMPORTER(MMD
        AssetLib/MMD/MMDCpp14.h
        AssetLib/MMD/MMDImporter.cpp
        AssetLib/MMD/MMDImporter.h
        AssetLib/MMD/MMDPmdParser.h
        AssetLib/MMD/MMDPmxParser.h
        AssetLib/MMD/MMDPmxParser.cpp
        AssetLib/MMD/MMDVmdParser.h
        )


# Sources
set(Core_SRCS Common/Assimp.cpp)

set(Logging_SRCS
        Common/Win32DebugLogStream.h
        Common/DefaultLogger.cpp
        Common/FileLogStream.h
        Common/StdOStreamLogStream.h
        )

set(Common_SRCS
        Common/BaseImporter.cpp
        Common/BaseProcess.cpp
        Common/BaseProcess.h
        Common/Importer.h
        Common/ScenePrivate.h
        Common/PostStepRegistry.cpp
        Common/ImporterRegistry.cpp
        Common/DefaultProgressHandler.h
        Common/DefaultIOStream.cpp
        Common/DefaultIOSystem.cpp
        Common/ZipArchiveIOSystem.cpp
        Common/PolyTools.h
        Common/Importer.cpp
        Common/IFF.h
        Common/SGSpatialSort.cpp
        Common/VertexTriangleAdjacency.cpp
        Common/VertexTriangleAdjacency.h
        Common/SpatialSort.cpp
        Common/SceneCombiner.cpp
        Common/ScenePreprocessor.cpp
        Common/ScenePreprocessor.h
        Common/SkeletonMeshBuilder.cpp
        Common/StandardShapes.cpp
        Common/TargetAnimation.cpp
        Common/TargetAnimation.h
        Common/RemoveComments.cpp
        Common/Subdivision.cpp
        Common/scene.cpp
        Common/Bitmap.cpp
        Common/Version.cpp
        Common/CreateAnimMesh.cpp
        Common/simd.h
        Common/simd.cpp
        Common/material.cpp
        Common/AssertHandler.cpp
        Common/Exceptional.cpp
        )

set(CApi_SRCS
        CApi/CInterfaceIOWrapper.cpp
        CApi/CInterfaceIOWrapper.h
        )

set(STEPParser_SRCS
        AssetLib/STEPParser/STEPFileReader.h
        AssetLib/STEPParser/STEPFileReader.cpp
        AssetLib/STEPParser/STEPFileEncoding.cpp
        AssetLib/STEPParser/STEPFileEncoding.h
        )

set(MaterialSystem_SRCS
        Material/MaterialSystem.cpp
        Material/MaterialSystem.h
        )

set(PostProcessing_SRCS
        PostProcessing/CalcTangentsProcess.cpp
        PostProcessing/CalcTangentsProcess.h
        PostProcessing/ComputeUVMappingProcess.cpp
        PostProcessing/ComputeUVMappingProcess.h
        PostProcessing/ConvertToLHProcess.cpp
        PostProcessing/ConvertToLHProcess.h
        PostProcessing/EmbedTexturesProcess.cpp
        PostProcessing/EmbedTexturesProcess.h
        PostProcessing/FindDegenerates.cpp
        PostProcessing/FindDegenerates.h
        PostProcessing/FindInstancesProcess.cpp
        PostProcessing/FindInstancesProcess.h
        PostProcessing/FindInvalidDataProcess.cpp
        PostProcessing/FindInvalidDataProcess.h
        PostProcessing/FixNormalsStep.cpp
        PostProcessing/FixNormalsStep.h
        PostProcessing/DropFaceNormalsProcess.cpp
        PostProcessing/DropFaceNormalsProcess.h
        PostProcessing/GenFaceNormalsProcess.cpp
        PostProcessing/GenFaceNormalsProcess.h
        PostProcessing/GenVertexNormalsProcess.cpp
        PostProcessing/GenVertexNormalsProcess.h
        PostProcessing/PretransformVertices.cpp
        PostProcessing/PretransformVertices.h
        PostProcessing/ImproveCacheLocality.cpp
        PostProcessing/ImproveCacheLocality.h
        PostProcessing/JoinVerticesProcess.cpp
        PostProcessing/JoinVerticesProcess.h
        PostProcessing/LimitBoneWeightsProcess.cpp
        PostProcessing/LimitBoneWeightsProcess.h
        PostProcessing/RemoveRedundantMaterials.cpp
        PostProcessing/RemoveRedundantMaterials.h
        PostProcessing/RemoveVCProcess.cpp
        PostProcessing/RemoveVCProcess.h
        PostProcessing/SortByPTypeProcess.cpp
        PostProcessing/SortByPTypeProcess.h
        PostProcessing/SplitLargeMeshes.cpp
        PostProcessing/SplitLargeMeshes.h
        PostProcessing/TextureTransform.cpp
        PostProcessing/TextureTransform.h
        PostProcessing/TriangulateProcess.cpp
        PostProcessing/TriangulateProcess.h
        PostProcessing/ValidateDataStructure.cpp
        PostProcessing/ValidateDataStructure.h
        PostProcessing/OptimizeGraph.cpp
        PostProcessing/OptimizeGraph.h
        PostProcessing/OptimizeMeshes.cpp
        PostProcessing/OptimizeMeshes.h
        PostProcessing/DeboneProcess.cpp
        PostProcessing/DeboneProcess.h
        PostProcessing/ProcessHelper.h
        PostProcessing/ProcessHelper.cpp
        PostProcessing/MakeVerboseFormat.cpp
        PostProcessing/MakeVerboseFormat.h
        PostProcessing/ScaleProcess.cpp
        PostProcessing/ScaleProcess.h
        PostProcessing/ArmaturePopulate.cpp
        PostProcessing/ArmaturePopulate.h
        PostProcessing/GenBoundingBoxesProcess.cpp
        PostProcessing/GenBoundingBoxesProcess.h
        PostProcessing/SplitByBoneCountProcess.cpp
        PostProcessing/SplitByBoneCountProcess.h
        )

# pugixml
set(Pugixml_SRCS
        contrib/pugixml/pugiconfig.hpp
        contrib/pugixml/pugixml.hpp
        )

# polyclipping
set(Clipper_SRCS
        contrib/clipper/clipper.hpp
        contrib/clipper/clipper.cpp
        )

# poly2tri
set(Poly2Tri_SRCS
        contrib/poly2tri/poly2tri/common/shapes.cc
        contrib/poly2tri/poly2tri/common/shapes.h
        contrib/poly2tri/poly2tri/common/utils.h
        contrib/poly2tri/poly2tri/sweep/advancing_front.h
        contrib/poly2tri/poly2tri/sweep/advancing_front.cc
        contrib/poly2tri/poly2tri/sweep/cdt.cc
        contrib/poly2tri/poly2tri/sweep/cdt.h
        contrib/poly2tri/poly2tri/sweep/sweep.cc
        contrib/poly2tri/poly2tri/sweep/sweep.h
        contrib/poly2tri/poly2tri/sweep/sweep_context.cc
        contrib/poly2tri/poly2tri/sweep/sweep_context.h
        )

# minizip/unzip
set(unzip_SRCS
        contrib/unzip/crypt.c
        contrib/unzip/crypt.h
        contrib/unzip/ioapi.c
        contrib/unzip/ioapi.h
        contrib/unzip/unzip.c
        contrib/unzip/unzip.h
        )

set(unzip_compile_SRCS ${unzip_SRCS})

# zip (https://github.com/kuba--/zip)
set(ziplib_SRCS
        contrib/zip/miniz.h
        contrib/zip/zip.c
        contrib/zip/zip.h
        )

# openddlparser
set(openddl_parser_SRCS
        contrib/openddlparser/code/OpenDDLParser.cpp
        contrib/openddlparser/code/DDLNode.cpp
        contrib/openddlparser/code/OpenDDLCommon.cpp
        contrib/openddlparser/code/OpenDDLExport.cpp
        contrib/openddlparser/code/Value.cpp
        contrib/openddlparser/code/OpenDDLStream.cpp
        contrib/openddlparser/include/openddlparser/OpenDDLParser.h
        contrib/openddlparser/include/openddlparser/OpenDDLParserUtils.h
        contrib/openddlparser/include/openddlparser/OpenDDLCommon.h
        contrib/openddlparser/include/openddlparser/OpenDDLExport.h
        contrib/openddlparser/include/openddlparser/OpenDDLStream.h
        contrib/openddlparser/include/openddlparser/DDLNode.h
        contrib/openddlparser/include/openddlparser/Value.h
        )

# Open3DGC
set(open3dgc_SRCS
        contrib/Open3DGC/o3dgcAdjacencyInfo.h
        contrib/Open3DGC/o3dgcArithmeticCodec.cpp
        contrib/Open3DGC/o3dgcArithmeticCodec.h
        contrib/Open3DGC/o3dgcBinaryStream.h
        contrib/Open3DGC/o3dgcCommon.h
        contrib/Open3DGC/o3dgcDVEncodeParams.h
        contrib/Open3DGC/o3dgcDynamicVectorDecoder.cpp
        contrib/Open3DGC/o3dgcDynamicVectorDecoder.h
        contrib/Open3DGC/o3dgcDynamicVectorEncoder.cpp
        contrib/Open3DGC/o3dgcDynamicVectorEncoder.h
        contrib/Open3DGC/o3dgcDynamicVector.h
        contrib/Open3DGC/o3dgcFIFO.h
        contrib/Open3DGC/o3dgcIndexedFaceSet.h
        contrib/Open3DGC/o3dgcIndexedFaceSet.inl
        contrib/Open3DGC/o3dgcSC3DMCDecoder.h
        contrib/Open3DGC/o3dgcSC3DMCDecoder.inl
        contrib/Open3DGC/o3dgcSC3DMCEncodeParams.h
        contrib/Open3DGC/o3dgcSC3DMCEncoder.h
        contrib/Open3DGC/o3dgcSC3DMCEncoder.inl
        contrib/Open3DGC/o3dgcTimer.h
        contrib/Open3DGC/o3dgcTools.cpp
        contrib/Open3DGC/o3dgcTriangleFans.cpp
        contrib/Open3DGC/o3dgcTriangleFans.h
        contrib/Open3DGC/o3dgcTriangleListDecoder.h
        contrib/Open3DGC/o3dgcTriangleListDecoder.inl
        contrib/Open3DGC/o3dgcTriangleListEncoder.h
        contrib/Open3DGC/o3dgcTriangleListEncoder.inl
        contrib/Open3DGC/o3dgcVector.h
        contrib/Open3DGC/o3dgcVector.inl
        )

set(zlib_SRCS
        contrib/zlib/adler32.c
        contrib/zlib/compress.c
        contrib/zlib/crc32.c
        contrib/zlib/deflate.c
        contrib/zlib/gzclose.c
        contrib/zlib/gzlib.c
        contrib/zlib/gzread.c
        contrib/zlib/gzwrite.c
        contrib/zlib/inflate.c
        contrib/zlib/infback.c
        contrib/zlib/inftrees.c
        contrib/zlib/inffast.c
        contrib/zlib/trees.c
        contrib/zlib/uncompr.c
        contrib/zlib/zutil.c
        )

set(assimp_src
        # Assimp Files
        ${Core_SRCS}
        ${CApi_SRCS}
        ${Common_SRCS}
        ${Logging_SRCS}
        # ${Exporter_SRCS}
        ${PostProcessing_SRCS}
        ${MaterialSystem_SRCS}
        ${STEPParser_SRCS}

        # Model Support
        ${ASSIMP_LOADER_SRCS}
        # ${ASSIMP_EXPORTER_SRCS}

        # Third-party libraries
        ${unzip_compile_SRCS}
        ${Poly2Tri_SRCS}
        ${Clipper_SRCS}
        ${openddl_parser_SRCS}
        ${open3dgc_SRCS}
        ${ziplib_SRCS}
        ${Pugixml_SRCS}

        ${zlib_SRCS}

        PARENT_SCOPE
        )

return()
