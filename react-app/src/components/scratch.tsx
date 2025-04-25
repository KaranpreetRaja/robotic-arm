import { useEffect, useState, useRef } from 'react';
import { DndProvider, useDrag, useDrop } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';

// Block Types
const BLOCK_TYPES = {
  MOVE_JOINT: 'MOVE_JOINT',
  MOVE_TO_POSITION: 'MOVE_TO_POSITION',
  WAIT: 'WAIT',
  LOOP: 'LOOP',
  GRIPPER: 'GRIPPER',
};

// Block Colors
const BLOCK_COLORS = {
  [BLOCK_TYPES.MOVE_JOINT]: 'bg-blue-500',
  [BLOCK_TYPES.MOVE_TO_POSITION]: 'bg-purple-500',
  [BLOCK_TYPES.WAIT]: 'bg-yellow-500',
  [BLOCK_TYPES.LOOP]: 'bg-orange-500',
  [BLOCK_TYPES.GRIPPER]: 'bg-green-500',
};

// Block Interface
interface Block {
  id: string;
  type: string;
  params: any;
  children?: Block[];
}

// Block Component Props
interface BlockProps {
  block: Block;
  removeBlock: (id: string) => void;
  updateBlock: (id: string, params: any) => void;
  index: number;
  moveBlock: (dragIndex: number, hoverIndex: number) => void;
}

// Single Block Component
const BlockComponent = ({ block, removeBlock, updateBlock, index, moveBlock }: BlockProps) => {
  const ref = useRef<HTMLDivElement>(null);

  const [{ isDragging }, drag] = useDrag({
    type: 'BLOCK',
    item: { index },
    collect: (monitor) => ({
      isDragging: monitor.isDragging(),
    }),
  });

  const [, drop] = useDrop({
    accept: 'BLOCK',
    hover(item: { index: number }, monitor) {
      if (!ref.current) return;
      const dragIndex = item.index;
      const hoverIndex = index;
      if (dragIndex === hoverIndex) return;

      moveBlock(dragIndex, hoverIndex);
      item.index = hoverIndex;
    },
  });

  drag(drop(ref));

  const renderBlockContent = () => {
    switch (block.type) {
      case BLOCK_TYPES.MOVE_JOINT:
        return (
          <div className="flex flex-col space-y-2 p-2">
            <div className="font-bold">Move Joints</div>
            <div className="grid grid-cols-2 gap-4">
              {block.params.angles.map((angle: number, idx: number) => (
                <div key={idx} className="flex items-center space-x-2">
                  <label className="whitespace-nowrap">Joint {idx + 1}:</label>
                  <input
                    type="number"
                    className="w-16 bg-white text-black rounded px-2 py-1 text-sm"
                    value={angle}
                    onChange={(e) => {
                      const newAngles = [...block.params.angles];
                      newAngles[idx] = parseFloat(e.target.value);
                      updateBlock(block.id, { angles: newAngles });
                    }}
                  />
                  <span>°</span>
                </div>
              ))}
            </div>
          </div>
        );

      case BLOCK_TYPES.MOVE_TO_POSITION:
        return (
          <div className="flex flex-col space-y-2 p-2">
            <div className="font-bold">Move To Position</div>
            <div className="flex space-x-2">
              <div className="flex items-center">
                <span className="mr-1">X:</span>
                <input
                  type="number"
                  className="w-14 text-black bg-white rounded px-2 py-1 text-sm"
                  value={block.params.x}
                  onChange={(e) => updateBlock(block.id, { ...block.params, x: parseFloat(e.target.value) })}
                  step="0.1"
                />
              </div>
              <div className="flex items-center">
                <span className="mr-1">Y:</span>
                <input
                  type="number"
                  className="w-14 text-black bg-white rounded px-2 py-1 text-sm"
                  value={block.params.y}
                  onChange={(e) => updateBlock(block.id, { ...block.params, y: parseFloat(e.target.value) })}
                  step="0.1"
                />
              </div>
              <div className="flex items-center">
                <span className="mr-1">Z:</span>
                <input
                  type="number"
                  className="w-14 text-black bg-white rounded px-2 py-1 text-sm"
                  value={block.params.z}
                  onChange={(e) => updateBlock(block.id, { ...block.params, z: parseFloat(e.target.value) })}
                  step="0.1"
                />
              </div>
            </div>
          </div>
        );

      case BLOCK_TYPES.WAIT:
        return (
          <div className="flex items-center p-2">
            <span className="font-bold mr-2">Wait</span>
            <input
              type="number"
              className="w-16 text-black bg-white rounded px-2 py-1 text-sm"
              value={block.params.duration}
              onChange={(e) => updateBlock(block.id, { ...block.params, duration: parseInt(e.target.value) })}
              min="0"
            />
            <span className="ml-1">seconds</span>
          </div>
        );

      case BLOCK_TYPES.LOOP:
        return (
          <div className="flex items-center p-2">
            <span className="font-bold mr-2">Repeat</span>
            <input
              type="number"
              className="w-16 bg-white text-black rounded px-2 py-1 text-sm"
              value={block.params.count}
              onChange={(e) => updateBlock(block.id, { ...block.params, count: parseInt(e.target.value) })}
              min="1"
            />
            <span className="ml-1">times</span>
          </div>
        );

      case BLOCK_TYPES.GRIPPER:
        return (
          <div className="flex items-center p-2">
            <span className="font-bold mr-2">Gripper</span>
            <select
              className="bg-white text-black rounded px-2 py-1 text-sm"
              value={block.params.action}
              onChange={(e) => updateBlock(block.id, { ...block.params, action: e.target.value })}
            >
              <option value="open">Open</option>
              <option value="close">Close</option>
            </select>
          </div>
        );

      default:
        return <div>Unknown Block Type</div>;
    }
  };

  return (
    <div
      ref={ref}
      className={`${BLOCK_COLORS[block.type]} rounded-lg text-white mb-2 shadow-md cursor-move ${isDragging ? 'opacity-50' : 'opacity-100'}`}
      style={{ width: '100%' }}
    >
      <div className="flex justify-between items-center">
        {renderBlockContent()}
        <button
          className="bg-red-500 hover:bg-red-600 text-white px-2 py-1 rounded-lg mr-2"
          onClick={() => removeBlock(block.id)}
        >
          X
        </button>
      </div>
    </div>
  );
};

// Block Menu Item Component
interface BlockMenuItemProps {
  type: string;
  label: string;
  addBlock: (type: string) => void;
}

const BlockMenuItem = ({ type, label, addBlock }: BlockMenuItemProps) => {
  return (
    <div
      className={`${BLOCK_COLORS[type]} rounded-lg text-white mb-2 p-2 cursor-pointer hover:brightness-90`}
      onClick={() => addBlock(type)}
    >
      {label}
    </div>
  );
};


export default function ScratchBlocks({ websocketPub, pubConnectionStatus }: { websocketPub: WebSocket | null, pubConnectionStatus: string }) {
  const [blocks, setBlocks] = useState<Block[]>([]);
  const [isRunning, setIsRunning] = useState(false);
  const [isCallbackEnabled, setIsCallbackEnabled] = useState(false)

  const addBlock = (type: string) => {
    const newBlock: Block = {
      id: `block-${Date.now()}`,
      type,
      params: getDefaultParams(type),
    };

    setBlocks([...blocks, newBlock]);
  };


  const getDefaultParams = (type: string) => {
    switch (type) {
      case BLOCK_TYPES.MOVE_JOINT:
        return { angles: [0, 0, 0, 0, 0, 0] };
      case BLOCK_TYPES.MOVE_TO_POSITION:
        return { x: 0, y: 0, z: 0 };
      case BLOCK_TYPES.WAIT:
        return { duration: 1 };
      case BLOCK_TYPES.LOOP:
        return { count: 5 };
      case BLOCK_TYPES.GRIPPER:
        return { action: 'open' };
      default:
        return {};
    }
  };


  const removeBlock = (id: string) => {
    setBlocks(blocks.filter(block => block.id !== id));
  };


  const updateBlock = (id: string, params: any) => {
    setBlocks(blocks.map(block =>
      block.id === id ? { ...block, params } : block
    ));
  };


  const moveBlock = (dragIndex: number, hoverIndex: number) => {
    const draggedBlock = blocks[dragIndex];
    const newBlocks = [...blocks];
    newBlocks.splice(dragIndex, 1);
    newBlocks.splice(hoverIndex, 0, draggedBlock);
    setBlocks(newBlocks);
  };

  const runProgram = async () => {
    if (!websocketPub || pubConnectionStatus !== 'connected') {
      console.error('WebSocket not connected');
      return;
    }

    setIsRunning(true);

    try {
      for (let i = 0; i < blocks.length; i++) {
        const block = blocks[i];
        await executeBlock(block);
      }
    } catch (error) {
      console.error('Error running program:', error);
    } finally {
      setIsRunning(false);
    }
  };

  const executeBlock = async (block: Block): Promise<void> => {
    if (!websocketPub) return;

    switch (block.type) {
      case BLOCK_TYPES.MOVE_JOINT:
        const orientation: Record<string, number> = {};
        block.params.angles.forEach((ang: number, i: number) => {
          orientation[`value${i + 1}`] = ang;
        });
        const jointMsg = {
          topic: "/robot/raw/joint_angles",
          message: JSON.stringify({ orientation, isCallbackEnabled }),
        };
        websocketPub.send(JSON.stringify(jointMsg));
        return new Promise(resolve => setTimeout(resolve, 500));


      case BLOCK_TYPES.MOVE_TO_POSITION:
        const positionMessage = {
          topic: "/robot/raw/target_pose",
          message: JSON.stringify({
            position: {
              x: block.params.x.toFixed(4),
              y: block.params.y.toFixed(4),
              z: block.params.z.toFixed(4)
            }
          })
        };
        websocketPub.send(JSON.stringify(positionMessage));
        return new Promise(resolve => setTimeout(resolve, 500));

      case BLOCK_TYPES.WAIT:
        return new Promise(resolve => setTimeout(resolve, block.params.duration * 1000));

      case BLOCK_TYPES.LOOP:
        for (let i = 0; i < block.params.count; i++) {
          if (block.children) {
            for (const child of block.children) {
              await executeBlock(child);
            }
          }
        }
        return Promise.resolve();

      case BLOCK_TYPES.GRIPPER:
        const gripperMessage = {
          topic: "/robot/gripper",
          message: JSON.stringify({
            action: block.params.action
          })
        };
        websocketPub.send(JSON.stringify(gripperMessage));
        return new Promise(resolve => setTimeout(resolve, 500));

      default:
        return Promise.resolve();
    }
  };

  return (
    <DndProvider backend={HTML5Backend}>
      <div className="flex flex-col h-full bg-white rounded-lg shadow-lg">
        <div className="bg-gray-800 text-white p-4 rounded-t-lg">
          <h2 className="text-xl font-bold">Block Programming</h2>
        </div>

        <div className="flex flex-1 overflow-hidden">
          {/* Block Menu */}
          <div className="w-1/4 bg-gray-100 p-4 overflow-y-auto">
            <h3 className="font-bold mb-2 text-lg">Blocks</h3>
            <BlockMenuItem type={BLOCK_TYPES.MOVE_JOINT} label="Move Joint" addBlock={addBlock} />
            <BlockMenuItem type={BLOCK_TYPES.MOVE_TO_POSITION} label="Move To Position" addBlock={addBlock} />
            <BlockMenuItem type={BLOCK_TYPES.WAIT} label="Wait" addBlock={addBlock} />
            <BlockMenuItem type={BLOCK_TYPES.LOOP} label="Repeat" addBlock={addBlock} />
            <BlockMenuItem type={BLOCK_TYPES.GRIPPER} label="Gripper" addBlock={addBlock} />
          </div>

          {/* Workspace */}
          <div className="flex-1 p-4 overflow-y-auto bg-gray-50">
            <div className="mb-4 flex space-x-2">
              <button
                className={`bg-green-500 hover:bg-green-600 text-white px-4 py-2 rounded-lg ${isRunning ? 'opacity-50 cursor-not-allowed' : ''}`}
                onClick={runProgram}
                disabled={isRunning || blocks.length === 0 || pubConnectionStatus !== 'connected'}
              >
                {isRunning ? 'Running...' : 'Run Program'}
              </button>

              <button
                className="bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded-lg"
                onClick={() => setBlocks([])}
                disabled={isRunning}
              >
                Clear All
              </button>

              <button
                className={`bg-yellow-500 hover:bg-yellow-600 text-black px-4 py-2 rounded-lg`}
                onClick={() => {
                  setIsCallbackEnabled((prev) => !prev)
                }

                }
              >
                {isCallbackEnabled ? 'Callback Enabled' : 'Callback Disabled'}
              </button>
            </div>

            <div className="bg-white border-2 border-dashed border-gray-300 rounded-lg min-h-full p-4">
              {blocks.length === 0 ? (
                <div className="text-center text-gray-400 py-8">
                  Drag blocks here to create your program
                </div>
              ) : (
                blocks.map((block, index) => (
                  <BlockComponent
                    key={block.id}
                    block={block}
                    removeBlock={removeBlock}
                    updateBlock={updateBlock}
                    index={index}
                    moveBlock={moveBlock}
                  />
                ))
              )}
            </div>
          </div>
        </div>
      </div>
    </DndProvider>
  );
}