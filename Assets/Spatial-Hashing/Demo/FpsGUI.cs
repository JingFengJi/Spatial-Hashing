using System;
using UnityEngine;

namespace HMH.ECS.SpatialHashing.Debug
{
    public class FpsGUI : MonoBehaviour
    {
        private FpsCounter m_FpsCounter;

        private void Start()
        {
            m_FpsCounter = new FpsCounter(0.5f);
        }

        private void Update()
        {
            m_FpsCounter.Update(Time.deltaTime, Time.unscaledDeltaTime);
        }
        
        private void OnGUI()
        {
            Matrix4x4 cachedMatrix = GUI.matrix;
            GUI.matrix = Matrix4x4.Scale(new Vector3(2, 2, 1f));
            string title = $"FPS: {m_FpsCounter.CurrentFps:F2}";
            if (GUILayout.Button(title, GUILayout.Width(150f), GUILayout.Height(100f)))
            {

            }

            GUI.matrix = cachedMatrix;
        }
    }
}