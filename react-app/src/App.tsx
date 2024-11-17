import { useState } from 'react'
import './App.css'
import MultiCameraViewer from './components/MultiCameraViewer';


function App() {
  const [count, setCount] = useState(0)

  return (
    <>
      <div>
        <MultiCameraViewer />
      </div>
    </>
  )
}

export default App
