using System;
using UnityEditor;
using UnityEngine;

namespace HandymanDatasetGenerator
{
    public static class DatasetSelfCheck
    {
        [MenuItem("Tools/Handyman Dataset Self Check")]
        public static void Run()
        {
            string[] accepted = { "apple", "apple#1", "apple#22", "apple_01", "apple(Clone)", "apple#1(Clone)" };
            string[] rejected = { "pineapple", "apple_tree", "apple#", "apple_", "applejuice", "apple#1/body" };
            foreach (string name in accepted)
                if (!HandymanDatasetWindow.MatchesClass(name, "apple")) throw new Exception("Expected match: " + name);
            foreach (string name in rejected)
                if (HandymanDatasetWindow.MatchesClass(name, "apple")) throw new Exception("Unexpected match: " + name);
            var frame = new HandymanDatasetWindow.Frame { cameraDistance = .7f, distanceBand = "near" };
            var restored = JsonUtility.FromJson<HandymanDatasetWindow.Frame>(JsonUtility.ToJson(frame));
            if (restored.distanceBand != "near" || Mathf.Abs(restored.cameraDistance - .7f) > .0001f)
                throw new Exception("Frame metadata roundtrip failed");
            var random = new System.Random(20260910);
            var cells = new bool[9];
            for (int i = 0; i < 1000; i++)
            {
                int cell;
                Vector2 uv = HandymanDatasetWindow.SampleViewport(random, out cell);
                cells[cell] = true;
                if (uv.x < .079f || uv.x > .921f || uv.y < .139f || uv.y > .861f)
                    throw new Exception("Viewport outside sampling limits");
                Quaternion rotation = HandymanDatasetWindow.ViewportRotation(uv, 46.82f, 640f / 480);
                Vector3 local = Quaternion.Inverse(rotation) * Vector3.forward;
                float tangent = Mathf.Tan(46.82f * Mathf.Deg2Rad * .5f);
                Vector2 projected = new Vector2(.5f + local.x / local.z / tangent / (640f / 480) / 2,
                    .5f + local.y / local.z / tangent / 2);
                if (Vector2.Distance(uv, projected) > .0001f)
                    throw new Exception("Viewport rotation projection mismatch");
            }
            foreach (bool covered in cells) if (!covered) throw new Exception("Missing viewport cell");
            Debug.Log("Handyman Dataset v3: naming, metadata and 1000 viewport projection checks PASS. This does not validate rendering or label alignment.");
        }
    }
}
